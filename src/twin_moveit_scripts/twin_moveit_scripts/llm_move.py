#!/usr/bin/env python3
"""
Drive the Panda from natural language, through a local LLM.

Each instruction takes one path: text -> Ollama, decoded against a JSON schema
-> parse -> clamp -> dispatch to a motion primitive.

The primitives are imported rather than invoked as subprocesses, so the whole
program shares one MoveItPy context. Building a context per command would pay
the construction cost and the action-client settle every time.

Two input paths, one dispatch:

    ros2 run twin_moveit_scripts llm_move
    cmd> go to the ready position

    ros2 topic pub --once /voice_command std_msgs/String "data: 'go home'"

The REPL blocks the main thread on input(), so the subscriber gets its own node
spun in a background thread. A lock serializes the motion itself, because both
paths drive the same MoveIt context.

Requires the Gazebo bringup and a running Ollama.
"""

import json
import threading
import urllib.request

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String

from twin_moveit_scripts.move_to import (
    move_to_named,
    move_to_pose,
    setup,
    shutdown,
)

OLLAMA_URL = 'http://localhost:11434/api/chat'

# A 3B model is a deliberate choice, not a compromise on a big machine: it fits
# entirely in 4 GB of VRAM alongside the simulator. Translating one sentence
# into one tool call does not need a larger model, and a 7B spills to CPU.
MODEL = 'qwen2.5:3b-instruct'

# Conservative reachable box in the panda_link0 frame, in metres. Deliberately
# smaller than the arm's true workspace: the cost of refusing a valid target is
# a retry, the cost of accepting an invalid one is a failed plan or a collision.
BOUNDS = {
    'x': (0.15, 0.65),
    'y': (-0.5, 0.5),
    'z': (0.15, 0.9),
}

SYSTEM_PROMPT = (
    'You translate a natural-language instruction for a Panda robot arm into '
    'ONE JSON tool call. Two tools:\n'
    '- move_to_named: go to a preset joint pose. '
    'args: {"name": "ready"|"home"}. Use for words like ready, home, reset.\n'
    '- move_to_pose: move the gripper to a Cartesian point. '
    'args: {"x": float, "y": float, "z": float} in meters.\n'
    'Pick move_to_named when the user names a preset pose; pick move_to_pose '
    'only when explicit x/y/z coordinates are given. Output JSON only.'
)

# Ollama constrains decoding to this schema, so the model cannot emit prose or
# an unexpected key - the shape is enforced while the tokens are generated
# rather than validated afterwards.
#
# The oneOf/const pairing is what makes it work at this model size. With a
# permissive {"args": {"type": "object"}} the 3B model read "go to ready" as a
# move_to_pose carrying an invented pose_name key: the schema allowed it, so
# constrained decoding dutifully produced it. Binding each action to its exact
# argument shape removes the option.
SCHEMA = {
    'oneOf': [
        {
            'type': 'object',
            'properties': {
                'action': {'const': 'move_to_named'},
                'args': {
                    'type': 'object',
                    'properties': {
                        'name': {'type': 'string', 'enum': ['ready', 'home']},
                    },
                    'required': ['name'],
                    'additionalProperties': False,
                },
            },
            'required': ['action', 'args'],
            'additionalProperties': False,
        },
        {
            'type': 'object',
            'properties': {
                'action': {'const': 'move_to_pose'},
                'args': {
                    'type': 'object',
                    'properties': {
                        'x': {'type': 'number'},
                        'y': {'type': 'number'},
                        'z': {'type': 'number'},
                    },
                    'required': ['x', 'y', 'z'],
                    'additionalProperties': False,
                },
            },
            'required': ['action', 'args'],
            'additionalProperties': False,
        },
    ]
}


def query_llm(text):
    """Turn one instruction into a parsed {action, args} tool call."""
    payload = {
        'model': MODEL,
        'stream': False,
        'messages': [
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': text},
        ],
        'format': SCHEMA,
        # Temperature 0: this is a classification, not a creative task, and the
        # same sentence should always produce the same call.
        'options': {'temperature': 0},
    }

    request = urllib.request.Request(
        OLLAMA_URL,
        data=json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        body = json.loads(response.read().decode())

    return json.loads(body['message']['content'])


def clamp_pose(args, logger):
    """
    Snap a Cartesian target into the reachable box, logging any change.

    The safety seam. A schema constrains the *shape* of what the model emits,
    never the *values*, so nothing upstream prevents it proposing a point two
    metres away. The model proposes; this decides.
    """
    clamped = {}
    for axis in ('x', 'y', 'z'):
        low, high = BOUNDS[axis]
        requested = float(args[axis])
        value = max(low, min(high, requested))
        if value != requested:
            logger.warn(
                f'clamped {axis}: {requested} -> {value} '
                f'(bounds {low}..{high})'
            )
        clamped[axis] = value
    return clamped


def handle_instruction(text, robot, arm, logger, params, lock, source='repl'):
    """
    Translate one instruction and drive the arm with it.

    Both input paths call this, so there is one translate-clamp-dispatch path
    rather than two that can drift apart. `source` tags the logs so it is
    visible which path drove a motion.

    The lock serializes the motion, not the translation: a second instruction
    arriving mid-move waits its turn rather than being dropped, which matches
    how the prompt already behaved. Two instructions can never drive the single
    MoveIt context at once.
    """
    text = text.strip()
    if not text:
        return

    try:
        call = query_llm(text)
    except Exception as error:
        logger.error(f'[{source}] LLM query failed: {error}')
        return

    action = call.get('action')
    args = call.get('args', {})
    logger.info(f'[{source}] LLM -> {action} {args}')

    with lock:
        if action == 'move_to_named':
            move_to_named(robot, arm, logger, params, args['name'])
        elif action == 'move_to_pose':
            safe = clamp_pose(args, logger)
            move_to_pose(
                robot, arm, logger, params, safe['x'], safe['y'], safe['z']
            )
        else:
            logger.error(f'[{source}] unknown action: {action}')


class VoiceCommandNode(Node):
    """Route each /voice_command String through the shared dispatch."""

    def __init__(self, robot, arm, logger, params, lock):
        super().__init__('llm_move_sub')
        self._robot = robot
        self._arm = arm
        self._logger = logger
        self._params = params
        self._lock = lock
        self.create_subscription(String, '/voice_command', self._on_message, 10)
        logger.info('Subscribed to /voice_command (std_msgs/String).')

    def _on_message(self, msg):
        handle_instruction(
            msg.data, self._robot, self._arm, self._logger, self._params,
            self._lock, source='topic',
        )


def start_topic_path(robot, arm, logger, params, lock):
    """Spin a subscriber node in the background, or report why it could not."""
    try:
        node = VoiceCommandNode(robot, arm, logger, params, lock)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin():
            # A thread that raises simply vanishes - no traceback, and the main
            # thread carries on looking healthy. The only symptom of losing this
            # thread is a topic with no subscriber, which is a long way from the
            # cause, so failures here are logged rather than swallowed.
            try:
                executor.spin()
            except Exception as error:
                logger.error(f'spin thread died: {error}')

        threading.Thread(target=spin, daemon=True).start()
        logger.info('Topic path started (spin thread live).')
        return executor, node
    except Exception as error:
        logger.error(
            f'could not start the /voice_command path: {error!r} '
            '-- the prompt still works'
        )
        return None, None


def main():
    robot, arm, logger, params = setup(node_name='llm_move')

    lock = threading.Lock()
    executor, sub_node = start_topic_path(robot, arm, logger, params, lock)

    logger.info(
        "LLM control ready. Type an instruction (Ctrl-D or 'quit' to exit). "
        'Also listening on /voice_command.'
    )

    try:
        while True:
            try:
                text = input('cmd> ')
            except EOFError:
                break
            if text.strip().lower() in ('quit', 'exit'):
                break
            handle_instruction(
                text, robot, arm, logger, params, lock, source='repl'
            )
    finally:
        logger.info('Shutting down.')
        if executor is not None:
            executor.shutdown()
        if sub_node is not None:
            sub_node.destroy_node()
        shutdown()


if __name__ == '__main__':
    main()
