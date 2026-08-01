#!/usr/bin/env python3
"""
Drive the Panda from natural language, through a local LLM.

Each instruction takes one path: text -> Ollama, decoded against a JSON schema
-> parse -> clamp -> dispatch to a motion primitive.

The primitives are imported rather than invoked as subprocesses, so the whole
program shares one MoveItPy context. Building a context per command would pay
the construction cost and the action-client settle every time.

    ros2 run twin_moveit_scripts llm_move
    cmd> go to the ready position
    cmd> move to x 0.3 y -0.2 z 0.5
    cmd> quit

Requires the Gazebo bringup and a running Ollama.
"""

import json
import urllib.request

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


def handle_instruction(text, robot, arm, logger, params):
    """Translate one instruction and drive the arm with it."""
    text = text.strip()
    if not text:
        return

    try:
        call = query_llm(text)
    except Exception as error:
        logger.error(f'LLM query failed: {error}')
        return

    action = call.get('action')
    args = call.get('args', {})
    logger.info(f'LLM -> {action} {args}')

    if action == 'move_to_named':
        move_to_named(robot, arm, logger, params, args['name'])
    elif action == 'move_to_pose':
        safe = clamp_pose(args, logger)
        move_to_pose(robot, arm, logger, params, safe['x'], safe['y'], safe['z'])
    else:
        logger.error(f'unknown action: {action}')


def main():
    robot, arm, logger, params = setup(node_name='llm_move')

    logger.info(
        "LLM control ready. Type an instruction (Ctrl-D or 'quit' to exit)."
    )

    try:
        while True:
            try:
                text = input('cmd> ')
            except EOFError:
                break
            if text.strip().lower() in ('quit', 'exit'):
                break
            handle_instruction(text, robot, arm, logger, params)
    finally:
        logger.info('Shutting down.')
        shutdown()


if __name__ == '__main__':
    main()
