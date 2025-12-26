"""
Launch file for wiz-server only.

Launches the WebSocket server that bridges ROS2 topics to the wiz frontend.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package lib directory
    pkg_dir = FindPackageShare('wiz')

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='WebSocket server port'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='WebSocket server host address'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (trace, debug, info, warn, error)'
    )

    # wiz-server process
    server_process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([pkg_dir, '..', 'lib', 'wiz', 'wiz-server'])
        ],
        name='wiz_server',
        output='screen',
        additional_env={
            'WIZ_PORT': LaunchConfiguration('port'),
            'WIZ_HOST': LaunchConfiguration('host'),
            'RUST_LOG': LaunchConfiguration('log_level'),
        }
    )

    return LaunchDescription([
        port_arg,
        host_arg,
        log_level_arg,
        server_process,
    ])
