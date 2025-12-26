"""
Launch file for wiz visualization tool.

Launches both wiz-server (WebSocket server) and wiz-frontend (GUI).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='ws://localhost:9090/ws',
        description='WebSocket server URL for frontend'
    )

    # wiz-server process
    server_process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([pkg_dir, '..', 'lib', 'wiz', 'wiz-server'])
        ],
        name='wiz_server',
        output='screen',
        env={
            'RUST_LOG': 'info',
        },
        additional_env={
            'WIZ_PORT': LaunchConfiguration('port'),
            'WIZ_HOST': LaunchConfiguration('host'),
        }
    )

    # wiz-frontend process (delayed start to wait for server)
    frontend_process = TimerAction(
        period=1.0,  # Wait 1 second for server to start
        actions=[
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution([pkg_dir, '..', 'lib', 'wiz', 'wiz-frontend'])
                ],
                name='wiz_frontend',
                output='screen',
                env={
                    'RUST_LOG': 'info',
                },
                additional_env={
                    'WIZ_SERVER_URL': LaunchConfiguration('server_url'),
                }
            )
        ]
    )

    return LaunchDescription([
        port_arg,
        host_arg,
        server_url_arg,
        server_process,
        frontend_process,
    ])
