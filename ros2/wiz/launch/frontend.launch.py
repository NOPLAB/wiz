"""
Launch file for wiz-frontend only.

Launches the wiz GUI frontend that connects to an existing wiz-server.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package lib directory
    pkg_dir = FindPackageShare('wiz')

    # Declare launch arguments
    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='ws://localhost:9090/ws',
        description='WebSocket server URL to connect to'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (trace, debug, info, warn, error)'
    )

    # wiz-frontend process
    frontend_process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([pkg_dir, '..', 'lib', 'wiz', 'wiz-frontend'])
        ],
        name='wiz_frontend',
        output='screen',
        additional_env={
            'WIZ_SERVER_URL': LaunchConfiguration('server_url'),
            'RUST_LOG': LaunchConfiguration('log_level'),
        }
    )

    return LaunchDescription([
        server_url_arg,
        log_level_arg,
        frontend_process,
    ])
