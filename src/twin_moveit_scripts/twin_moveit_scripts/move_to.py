#!/usr/bin/env python3
"""
Scripted motion primitives for the Panda, driven by moveit_py.

    ros2 run twin_moveit_scripts move_to named ready
    ros2 run twin_moveit_scripts move_to pose 0.28 -0.2 0.5
    ros2 run twin_moveit_scripts move_to demo          # ready, then home

moveit_py embeds a MoveIt context in this process rather than talking to a
standalone move_group. It does not connect to one - it *is* one. So this must
not be run alongside move_group_gazebo.launch.py: two contexts would fight over
the same controllers.

The Gazebo bringup must already be running, because execution needs
arm_controller to exist.
"""

import sys
import time

from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.utils import create_params_file_from_dict

from moveit_configs_utils import MoveItConfigsBuilder

import rclpy
from rclpy.logging import get_logger

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

    logger.info('MoveItPy up; panda_arm ready.')
    time.sleep(EXECUTION_CLIENT_SETTLE)
    return robot, arm, logger, params


def shutdown():
    rclpy.shutdown()


def plan_and_execute(robot, arm, logger, params):
    """Plan to whatever goal is already set, then execute it."""
    try:
        result = arm.plan(single_plan_parameters=params)
    except Exception as error:
        logger.error(f'Planning raised: {error}')
        return False

    if not result:
        logger.error('Planning FAILED.')
        return False

    logger.info('Planning succeeded, executing...')
    try:
        # An empty controller list lets MoveIt choose from the controllers
        # declared in moveit_controllers.yaml, which is how RViz executes too.
        robot.execute(result.trajectory, controllers=[])
    except Exception as error:
        logger.error(f'Execution raised: {error}')
        return False

    time.sleep(MOTION_SETTLE)
    return True


def move_to_named(robot, arm, logger, params, name):
    """Move to a pose named in the SRDF. Joint-space, so no IK is involved."""
    logger.info(f'=== move_to_named({name!r}) ===')
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    ok = plan_and_execute(robot, arm, logger, params)
    logger.info(f'move_to_named({name!r}) -> {"ok" if ok else "FAILED"}')
    return ok


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
    ok = plan_and_execute(robot, arm, logger, params)
    logger.info(f'move_to_pose -> {"ok" if ok else "FAILED"}')
    return ok


def main():
    # ros2 run appends its own --ros-args; drop them before reading ours.
    argv = [arg for arg in sys.argv[1:] if not arg.startswith('--')]

    robot, arm, logger, params = setup()

    command = argv[0] if argv else 'demo'

    if command == 'named':
        move_to_named(robot, arm, logger, params, argv[1])
    elif command == 'pose':
        move_to_pose(
            robot, arm, logger, params,
            float(argv[1]), float(argv[2]), float(argv[3]),
        )
    else:
        for name in ('ready', 'home'):
            move_to_named(robot, arm, logger, params, name)

    logger.info('Done.')
    shutdown()


if __name__ == '__main__':
    main()
