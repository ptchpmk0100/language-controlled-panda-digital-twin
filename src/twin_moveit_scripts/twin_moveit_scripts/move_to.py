#!/usr/bin/env python3
"""
Scripted motion primitives for the Panda, driven by moveit_py.

    ros2 run twin_moveit_scripts move_to named ready
    ros2 run twin_moveit_scripts move_to pose 0.28 -0.2 0.5
    ros2 run twin_moveit_scripts move_to grip 0.0      # close
    ros2 run twin_moveit_scripts move_to grip 0.04     # open
    ros2 run twin_moveit_scripts move_to demo          # ready, then home

moveit_py embeds a MoveIt context in this process rather than talking to a
standalone move_group. It does not connect to one - it *is* one. So this must
not be run alongside move_group_gazebo.launch.py: two contexts would fight over
the same controllers.

The Gazebo bringup must already be running, because execution needs
arm_controller to exist.
"""

from dataclasses import dataclass
import sys
import time
from typing import Optional, Tuple

from control_msgs.action import GripperCommand

from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.utils import create_params_file_from_dict

from moveit_configs_utils import MoveItConfigsBuilder

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node

from sensor_msgs.msg import JointState

ARM_GROUP = 'panda_arm'

# Cartesian goals are expressed for this link, and framed in this one.
END_EFFECTOR_LINK = 'panda_link8'
PLANNING_FRAME = 'panda_link0'

PLANNING_PIPELINE = 'ompl'
PLANNER_ID = 'RRTConnect'

# Seconds. The trajectory-execution action client needs a moment to connect to
# arm_controller after the context is built. Executing before it does fails with
# "Action client not connected", and `ros2 run` starts fast enough to hit it.
EXECUTION_CLIENT_SETTLE = 2.0

# Seconds. Lets the arm finish moving before anything reads its state back.
MOTION_SETTLE = 3.0

# The gripper is an action, not a plan. panda_finger_joint1 is prismatic and
# its value is the travel of a single finger, in metres - not half-width and
# not total width. The URDF limits it to [0.0, 0.04].
GRIPPER_ACTION = '/gripper_controller/gripper_cmd'
FINGER_JOINT = 'panda_finger_joint1'
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04

# Newtons. A GripperCommand max_effort of zero or less means "no limit" in most
# implementations, so a real positive default is passed and callers can lower
# it for a delicate grasp.
GRIPPER_DEFAULT_EFFORT = 10.0

# Seconds. The controller returns the instant it declares the goal reached,
# while the prismatic finger is still coasting. Sampling immediately reads a
# mid-flight value; this is the finger's equivalent of MOTION_SETTLE.
GRIPPER_SETTLE = 0.5


@dataclass
class MoveResult:
    """
    What a motion primitive reports back.

    Chosen over a bool, which cannot carry why or where, and over a
    (bool, dict) tuple, whose keys are a convention that rots the first time a
    field is added. New fields go here without touching any call site.

    `stage` is the distinction anything reacting to a failure needs: a plan
    that never existed calls for a different target, an execution that broke
    mid-motion calls for stopping and looking.
    """

    ok: bool
    stage: str                     # 'executed' | 'plan_failed' | 'exec_error'
    final_pose: Optional[Tuple[float, float, float]] = None
    error: Optional[str] = None

    def __str__(self):
        if self.ok and self.final_pose is not None:
            x, y, z = self.final_pose
            return f'ok @ ({x:.3f}, {y:.3f}, {z:.3f})'
        return f'FAILED [{self.stage}]: {self.error}'


@dataclass
class GripResult:
    """
    What the gripper primitive reports back.

    A sibling to MoveResult rather than folded into it. `stalled` and
    `final_width` mean nothing for an arm move, and `final_pose` means nothing
    for a grip; one type carrying all four would be a shape that lies about
    half its uses.

    `stalled` is the grasp predicate and `final_width` is a diagnostic. They
    are separate fields because they answer different questions: whether
    something is held, and where the fingers stopped. Crucially, stalled=True
    is SUCCESS - the fingers closed until they met resistance.
    """

    ok: bool
    stage: str                     # 'reached' | 'stalled' | 'rejected' | 'error'
    stalled: bool = False
    final_width: Optional[float] = None
    error: Optional[str] = None

    def __str__(self):
        if self.ok:
            width = (
                f'{self.final_width:.4f}'
                if self.final_width is not None else '?'
            )
            held = ' HELD(stalled)' if self.stalled else ''
            return f'ok [{self.stage}] width={width}{held}'
        return f'FAILED [{self.stage}]: {self.error}'


def build_config_dict():
    """
    Assemble the MoveIt configuration in the layout moveit_cpp expects.

    The builder produces `planning_pipelines` as a flat list of names, but
    moveit_cpp reads the names from `planning_pipelines.pipeline_names` while
    reading each pipeline's plugin configuration from its own *top-level*
    namespace. Both shapes have to be present at once, so only the names are
    nested and the top-level `ompl` block is left where it is.

    Restricting to OMPL is deliberate: the builder otherwise discovers CHOMP,
    STOMP and Pilz, and moveit_cpp aborts if any configured pipeline fails to
    load.
    """
    configs = MoveItConfigsBuilder(
        'panda', package_name='twin_moveit_config'
    ).planning_pipelines(
        pipelines=[PLANNING_PIPELINE],
        default_planning_pipeline=PLANNING_PIPELINE,
    ).to_moveit_configs()

    config = configs.to_dict()

    pipeline_names = config.get('planning_pipelines')
    if isinstance(pipeline_names, list):
        config['planning_pipelines'] = {'pipeline_names': pipeline_names}

    config['use_sim_time'] = True
    return config


def make_plan_parameters(robot):
    """
    Name the pipeline and planner explicitly.

    Without these, plan() reports "No planning pipeline available for name ''":
    the request carries no pipeline, and there is no implicit default.
    """
    params = PlanRequestParameters(robot, PLANNING_PIPELINE)
    params.planning_pipeline = PLANNING_PIPELINE
    params.planner_id = PLANNER_ID
    params.planning_time = 5.0
    params.planning_attempts = 5
    params.max_velocity_scaling_factor = 0.1
    params.max_acceleration_scaling_factor = 0.1
    return params


def _on_joint_states(msg):
    """Cache the live finger value. Indexed by name, never by position."""
    try:
        index = msg.name.index(FINGER_JOINT)
    except ValueError:
        return
    _on_joint_states.latest = float(msg.position[index])


_on_joint_states.latest = None


def setup(node_name='move_to'):
    """Build the one MoveIt context for this process."""
    rclpy.init()
    logger = get_logger(node_name)

    # The configuration reaches the embedded moveit_cpp node through a params
    # file rooted at '/**'. Passing it as config_dict= instead makes
    # use_sim_time trip a qos_overrides error at construction; scoping it to
    # this node's name leaves the embedded node on wall-clock, which shows up
    # later as joint states that always look a second stale and an execution
    # that refuses to validate. The wildcard root reaches every node in the
    # process, which is the one thing that works.
    params_file = create_params_file_from_dict(build_config_dict(), '/**')
    robot = MoveItPy(node_name=node_name, launch_params_filepaths=[params_file])

    arm = robot.get_planning_component(ARM_GROUP)
    params = make_plan_parameters(robot)

    # A node of our own, separate from the one MoveItPy hides and already
    # spins. The gripper needs a spinnable node for its ActionClient, and
    # spinning MoveItPy's would double-spin its current-state monitor and
    # deadlock. This node is also the natural home for the non-MoveIt
    # interfaces that grasping and perception will need later.
    twin = Node('twin_gripper')
    twin.create_subscription(JointState, '/joint_states', _on_joint_states, 10)
    twin.gripper_client = ActionClient(twin, GripperCommand, GRIPPER_ACTION)

    logger.info('MoveItPy up; panda_arm ready. twin_gripper node up.')
    time.sleep(EXECUTION_CLIENT_SETTLE)
    return robot, arm, logger, params, twin


def shutdown(twin=None):
    if twin is not None:
        try:
            twin.destroy_node()
        except Exception:
            pass
    rclpy.shutdown()


def read_end_effector_xyz(robot, logger):
    """
    Read where the end effector actually is, from the live current state.

    Forward kinematics on the robot's current state, not an echo of the goal.
    The two diverge in exactly the cases a caller would want to react to - a
    clamped target, drift, a partially executed motion - so echoing the request
    would defeat the purpose of reporting a pose at all.

    Returns None on failure. A diagnostic must never be able to turn a
    successful motion into a reported failure.
    """
    try:
        state = robot.get_planning_component(ARM_GROUP).get_start_state()
        transform = state.get_global_link_transform(END_EFFECTOR_LINK)
        # A 4x4 homogeneous transform; the translation is its last column.
        return (
            float(transform[0][3]),
            float(transform[1][3]),
            float(transform[2][3]),
        )
    except Exception as error:
        logger.warn(f'could not read live end-effector pose: {error}')
        return None


def plan_and_execute(robot, arm, logger, params):
    """Plan to whatever goal is already set, execute it, and report back."""
    try:
        result = arm.plan(single_plan_parameters=params)
    except Exception as error:
        logger.error(f'Planning raised: {error}')
        return MoveResult(ok=False, stage='plan_failed', error=str(error))

    if not result:
        logger.error('Planning FAILED.')
        return MoveResult(
            ok=False, stage='plan_failed',
            error='planner returned no trajectory',
        )

    logger.info('Planning succeeded, executing...')
    try:
        # An empty controller list lets MoveIt choose from the controllers
        # declared in moveit_controllers.yaml, which is how RViz executes too.
        robot.execute(result.trajectory, controllers=[])
    except Exception as error:
        logger.error(f'Execution raised: {error}')
        time.sleep(MOTION_SETTLE)
        return MoveResult(
            ok=False, stage='exec_error', error=str(error),
            final_pose=read_end_effector_xyz(robot, logger),
        )

    # Sampled after the settle, so this is the resting pose rather than a
    # point somewhere mid-motion.
    time.sleep(MOTION_SETTLE)
    return MoveResult(
        ok=True, stage='executed',
        final_pose=read_end_effector_xyz(robot, logger),
    )


def _spin_until(executor, future, timeout):
    """Spin the given executor until the future completes or time runs out."""
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)


def read_finger_width(twin, logger, executor=None):
    """
    Read the finger's actual value live from /joint_states.

    The exact mirror of read_end_effector_xyz: report where the finger is, not
    the width that was asked for. The cache is cleared first so a genuinely
    fresh sample is taken rather than a stale one.

    A node can belong to only one executor at a time, so a caller that already
    holds `twin` passes its own executor in. Returns None on a read miss, so a
    failed read never masquerades as a failed grasp.
    """
    owned = executor is None
    try:
        if owned:
            executor = SingleThreadedExecutor()
            executor.add_node(twin)

        _on_joint_states.latest = None
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if _on_joint_states.latest is not None:
                break
        return _on_joint_states.latest
    except Exception as error:
        logger.warn(f'could not read live finger width: {error}')
        return None
    finally:
        if owned and executor is not None:
            executor.remove_node(twin)


def move_gripper(twin, logger, width, max_effort=GRIPPER_DEFAULT_EFFORT):
    """
    Command the gripper and wait for the controller's verdict.

    An action round-trip, not a MoveIt plan: no start state, no planner, no
    trajectory. `width` is the target value of the finger joint, clamped to the
    URDF limit.

    stalled=True in the result is grasp SUCCESS - the fingers closed until they
    met resistance. allow_stalling in the controller configuration is what
    makes it arrive as a reached-with-stall rather than an abort.
    """
    target = max(GRIPPER_MIN, min(GRIPPER_MAX, float(width)))
    if target != float(width):
        logger.warn(
            f'clamped gripper width: {width} -> {target} '
            f'(limit {GRIPPER_MIN}..{GRIPPER_MAX})'
        )
    logger.info(f'=== move_gripper(width={target}, effort={max_effort}) ===')

    client = twin.gripper_client
    executor = SingleThreadedExecutor()
    executor.add_node(twin)

    try:
        if not client.wait_for_server(timeout_sec=5.0):
            return GripResult(
                ok=False, stage='error',
                error=f'{GRIPPER_ACTION} action server not available',
            )

        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = float(max_effort)

        send_future = client.send_goal_async(goal)
        _spin_until(executor, send_future, timeout=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return GripResult(
                ok=False, stage='rejected',
                error='goal rejected by the controller',
            )

        result_future = handle.get_result_async()
        _spin_until(executor, result_future, timeout=15.0)
        wrapped = result_future.result()
        if wrapped is None:
            return GripResult(
                ok=False, stage='error',
                error='no result returned before timeout',
            )

        result = wrapped.result
        stalled = bool(getattr(result, 'stalled', False))

        # Let the finger settle before sampling. The controller returns the
        # instant it declares the goal reached, while the joint is still
        # coasting; sampling immediately reads a mid-flight value.
        deadline = time.monotonic() + GRIPPER_SETTLE
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        live = read_finger_width(twin, logger, executor=executor)
        final_width = (
            live if live is not None
            else float(getattr(result, 'position', target))
        )

        grip = GripResult(
            ok=True,
            stage='stalled' if stalled else 'reached',
            stalled=stalled,
            final_width=final_width,
        )
        logger.info(f'move_gripper -> {grip}')
        return grip
    except Exception as error:
        logger.error(f'move_gripper raised: {error}')
        return GripResult(ok=False, stage='error', error=str(error))
    finally:
        executor.remove_node(twin)


def move_to_named(robot, arm, logger, params, name):
    """Move to an SRDF pose. Joint-space, so no IK is involved."""
    logger.info(f'=== move_to_named({name!r}) ===')
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    result = plan_and_execute(robot, arm, logger, params)
    logger.info(f'move_to_named({name!r}) -> {result}')
    return result


def move_to_pose(robot, arm, logger, params, x, y, z):
    """Move the end-effector link to a Cartesian point. Solved with IK."""
    logger.info(f'=== move_to_pose(x={x}, y={y}, z={z}) ===')

    pose = PoseStamped()
    pose.header.frame_id = PLANNING_FRAME
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link=END_EFFECTOR_LINK)
    result = plan_and_execute(robot, arm, logger, params)
    logger.info(f'move_to_pose -> {result}')
    return result


def main():
    # ros2 run appends its own --ros-args; drop them before reading ours.
    argv = [arg for arg in sys.argv[1:] if not arg.startswith('--')]

    robot, arm, logger, params, twin = setup()

    command = argv[0] if argv else 'demo'

    if command == 'named':
        move_to_named(robot, arm, logger, params, argv[1])
    elif command == 'pose':
        move_to_pose(
            robot, arm, logger, params,
            float(argv[1]), float(argv[2]), float(argv[3]),
        )
    elif command == 'grip':
        effort = float(argv[2]) if len(argv) > 2 else GRIPPER_DEFAULT_EFFORT
        move_gripper(twin, logger, float(argv[1]), max_effort=effort)
    else:
        for name in ('ready', 'home'):
            move_to_named(robot, arm, logger, params, name)

    logger.info('Done.')
    shutdown(twin)


if __name__ == '__main__':
    main()
