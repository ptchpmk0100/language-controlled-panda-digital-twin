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
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
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

    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'),
        'urdf',
        'panda.urdf',
    ])

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
        spawn,
        bridge,
    ])
