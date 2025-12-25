#!/usr/bin/env python3
"""
ROS2 Example Publisher for wiz visualization testing.

This script publishes sample sensor data for testing wiz with ROS2:
- LaserScan messages on /scan
- PointCloud2 messages on /velodyne_points

Usage:
    ros2 run py_pubsub ros2_publisher
    or
    python3 ros2_publisher.py
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


class WizExamplePublisher(Node):
    def __init__(self):
        super().__init__('wiz_example_publisher')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/velodyne_points', qos)

        # Timer for publishing at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tick = 0

        self.get_logger().info('wiz example publisher started')
        self.get_logger().info('Publishing: /scan (LaserScan), /velodyne_points (PointCloud2)')

    def timer_callback(self):
        self.tick += 1
        now = self.get_clock().now().to_msg()

        self.publish_laser_scan(now)
        self.publish_point_cloud(now)

    def publish_laser_scan(self, stamp):
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = math.pi / 180.0  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 30.0

        num_rays = 360
        time_factor = self.tick * 0.05

        ranges = []
        intensities = []

        for i in range(num_rays):
            angle = msg.angle_min + i * msg.angle_increment
            # Create moving obstacles
            base_range = 5.0 + 2.0 * math.sin(angle * 3.0 + time_factor)

            # Add some "obstacles"
            if abs((angle - time_factor * 0.2) % (2.0 * math.pi)) < 0.3:
                base_range = min(base_range, 2.0)

            ranges.append(max(0.1, min(30.0, base_range)))
            intensities.append(1.0 - (ranges[-1] / 30.0))

        msg.ranges = ranges
        msg.intensities = intensities

        self.scan_pub.publish(msg)

    def publish_point_cloud(self, stamp):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        num_points = 5000
        point_step = 16  # x, y, z (12 bytes) + padding (4 bytes)

        msg.height = 1
        msg.width = num_points
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * num_points

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        time_factor = self.tick * 0.02
        data = bytearray()

        for i in range(num_points):
            t = i / num_points
            angle = t * 2.0 * math.pi * 10.0 + time_factor
            radius = 3.0 + 2.0 * math.sin(t * 4.0 * math.pi + time_factor * 0.5)

            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = t * 3.0 + 0.5 * math.sin(angle * 2.0)

            data.extend(struct.pack('<fff', x, y, z))
            data.extend(bytes(4))  # padding

        msg.data = bytes(data)

        self.pointcloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WizExamplePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
