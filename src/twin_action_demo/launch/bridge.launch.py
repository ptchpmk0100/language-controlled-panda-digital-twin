"""
Bridge every Panda topic between Gazebo and ROS 2, from a YAML config.

Kept separate from the full bringup so it can be launched on its own while
debugging: if commands are not reaching the robot, running just this file
answers whether the bridge is the problem.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Resolved from the install space at launch time, so no absolute path to
    # anyone's home directory ends up in the file.
    bridge_config = PathJoinSubstitution([
        FindPackageShare('twin_action_demo'),
        'config',
        'panda_bridge.yaml',
    ])

    # The config file arrives as a ROS *parameter*, not as a command-line
    # argument. Inline `arguments=[...]` specs are the other way to drive this
    # same node; a YAML table scales past a couple of topics far better.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    return LaunchDescription([bridge])
