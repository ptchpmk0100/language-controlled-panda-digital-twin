"""
Launch the move_to node with no arguments, which runs its demo sequence.

The node builds its own MoveIt configuration and delivers it to the embedded
context through a params file, so nothing is passed in here. For the `named`
and `pose` commands use `ros2 run`, which forwards positional arguments
cleanly.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    move_to = Node(
        package='twin_moveit_scripts',
        executable='move_to',
        output='screen',
    )

    return LaunchDescription([move_to])
