"""
Run move_group against the existing Gazebo control stack.

Not to be confused with the generated demo.launch.py, which stands up fake
hardware and its own controller manager. The robot and its controllers already
exist; this launches the planner alone and lets it drive them.

Two things it deliberately does not do: start robot_state_publisher, and spawn
controllers. panda_bringup.launch.py already owns both, and doing it twice
produces two publishers of /robot_description and a spawner fighting over
joints that are already claimed.

Run after panda_bringup.launch.py, once the controllers report active.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # The URDF is resolved through .setup_assistant to twin_description's
    # panda.urdf - the same world-anchored description Gazebo simulates. A
    # different Panda here would put the planner and the simulator in different
    # frames while both looked healthy.
    moveit_config = MoveItConfigsBuilder(
        'panda', package_name='twin_moveit_config'
    ).to_moveit_configs()

    # Copied from the stock generator: without these the planning scene and the
    # semantic description are never published, and RViz shows an empty world.
    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # The node is assembled by hand rather than through
    # generate_move_group_launch, which offers no hook for use_sim_time. That
    # parameter is not optional here: everything else runs on Gazebo's clock,
    # and a planner on wall-clock produces trajectory timestamps the controller
    # rejects. Parameter dictionaries merge last-wins, so it goes last.
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([move_group_node])
