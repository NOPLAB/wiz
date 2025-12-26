"""
Launch file for wiz demo.

Launches wiz along with demo publisher nodes to showcase visualization capabilities.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include wiz launch
    wiz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('wiz'), '/launch/wiz.launch.py'
        ]),
    )

    # Demo marker publisher
    marker_publisher = Node(
        package='wiz_example',
        executable='demo_marker_publisher',
        name='demo_marker_publisher',
        output='screen',
    )

    # Demo pointcloud publisher
    pointcloud_publisher = Node(
        package='wiz_example',
        executable='demo_pointcloud_publisher',
        name='demo_pointcloud_publisher',
        output='screen',
    )

    # Demo TF broadcaster
    tf_broadcaster = Node(
        package='wiz_example',
        executable='demo_tf_broadcaster',
        name='demo_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([
        wiz_launch,
        marker_publisher,
        pointcloud_publisher,
        tf_broadcaster,
    ])
