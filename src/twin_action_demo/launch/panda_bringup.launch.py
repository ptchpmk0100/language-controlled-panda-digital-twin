"""
Bring the whole Panda twin up with one command, from a plain ROS terminal.

Starts Gazebo already running, spawns the Panda, and includes the bridge. The
vendored Gazebo environment is set here rather than being sourced beforehand,
so this file needs no `gz_env.sh` and no manual press of play.
"""

from glob import glob

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Gazebo ships inside the ROS tree as `-vendor` packages rather than on the
# system PATH, so these have to be discovered the same way gz_env.sh does.
# The glob is '*_vendor', not 'gz_*_vendor': sdformat_vendor supplies
# libsdformat and does not carry the gz_ prefix.
GZ_CONFIG_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz')))
GZ_LIB_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/lib')))
GZ_TOOLS_BIN = '/opt/ros/jazzy/opt/gz_tools_vendor/bin'


def generate_launch_description():
    # Set overwrites, Append preserves. GZ_VERSION and GZ_CONFIG_PATH are ours
    # to define; PATH and LD_LIBRARY_PATH belong to the caller and must never
    # be clobbered. These are listed first so they apply before Gazebo starts.
    set_gz_version = SetEnvironmentVariable('GZ_VERSION', 'harmonic')
    set_gz_config = SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATHS)
    append_library_path = AppendEnvironmentVariable(
        'LD_LIBRARY_PATH', GZ_LIB_PATHS
    )
    append_path = AppendEnvironmentVariable('PATH', GZ_TOOLS_BIN)

    # '-r' starts the world running. It is what replaces clicking play, and
    # without it the controllers never step and every command appears ignored.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # Gazebo keeps simulation time internally and does not publish it to ROS.
    # A bridge has to carry it across, and without one /clock has zero
    # publishers and every use_sim_time node waits forever for a clock that
    # never arrives. '[' is the gz-to-ROS direction: the simulator owns time.
    #
    # Deliberately separate from bridge.launch.py, which is vestigial under
    # ros2_control, so that deleting it later stays a clean delete with nothing
    # load-bearing hidden inside.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'),
        'urdf',
        'panda.urdf',
    ])

    # Required, not cosmetic. gz_ros2_control reads the robot description off
    # the /robot_description topic to initialise its hardware interface; with
    # no publisher, the controller_manager waits on that topic forever and the
    # spawners never find a manager to talk to.
    # `cat` because this is plain URDF; an xacro source would use `xacro` here.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_path]), value_type=str
                ),
            },
            # Must match the controller manager. A split clock - one node on
            # sim-time, the other on wall-clock - desynchronises TF.
            {'use_sim_time': True},
        ],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'panda',
            '-file', urdf_path,
            '-z', '0.0',
        ],
        output='screen',
    )

    # Controllers are registered in the YAML but not started by it. A spawner
    # loads and activates each one, and it can only do that once the manager
    # exists inside the running simulation. Chaining on process exit sequences
    # them instead of racing: spawn finishes, then the broadcaster, then the
    # position controller.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    arm_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    gripper_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # finger_state_publisher is deliberately NOT started any more. It existed
    # only because the fingers had no command interface, so the broadcaster
    # never reported them and move_group's scene monitor blocked on an
    # incomplete robot. Now that the finger joint is in <ros2_control>, the
    # broadcaster reports it with real values - and the stub would be a second
    # publisher of the same joint, asserting a fixed 0.02 m against whatever
    # the finger is actually doing. A placeholder must not outlive the signal
    # it stood in for.

    # Included rather than duplicated, so the bridge stays independently
    # launchable and there is only one copy of its configuration.
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('twin_action_demo'),
                'launch',
                'bridge.launch.py',
            ])
        )
    )

    return LaunchDescription([
        set_gz_version,
        set_gz_config,
        append_library_path,
        append_path,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn,
        broadcaster_after_spawn,
        arm_controller_after_broadcaster,
        gripper_controller_after_broadcaster,
        bridge,
    ])
