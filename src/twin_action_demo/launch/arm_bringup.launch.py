"""
Bring up the one-joint arm: spawn, bridge, and action server.

Gazebo itself is deliberately not started here. The simulator needs the
vendored Gazebo environment variables exported into its terminal, and those are
kept out of ROS-only terminals on purpose (see docs/learning/step-02). Start
``gz sim empty.sdf`` separately and press play, then run this file.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WORLD_NAME = 'empty'
MODEL_NAME = 'one_joint_arm'

JOINT_STATE_TOPIC = f'/world/{WORLD_NAME}/model/{MODEL_NAME}/joint_state'
COMMAND_TOPIC = f'/model/{MODEL_NAME}/joint/joint1/cmd_pos'


def generate_launch_description():
    # A substitution is a promise to compute a value later: it is resolved when
    # the launch system starts the nodes, not while this function runs. That is
    # why the path cannot be assembled with os.path.join here. FindPackageShare
    # searches the install space, so twin_description must be built and sourced.
    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'),
        'urdf',
        'one_joint_arm.urdf',
    ])

    # One-shot: sends a spawn request to the running simulator and exits. Its
    # clean exit is not a crash.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', WORLD_NAME,
            '-name', MODEL_NAME,
            '-file', urdf_path,
            '-z', '0.0',
        ],
        output='screen',
    )

    # Both directions of the loop in one process. In a bridge specification the
    # bracket encodes direction: '[' is gz to ROS, ']' is ROS to gz. A comma
    # separates mappings, so a single mapping must contain none.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{JOINT_STATE_TOPIC}@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'{COMMAND_TOPIC}@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    server = Node(
        package='twin_action_demo',
        executable='move_joint_server',
        output='screen',
    )

    return LaunchDescription([spawn, bridge, server])
