#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "============================================="
echo "  wiz_example Demo Nodes"
echo "============================================="
echo ""
echo "Starting demo nodes..."
echo "  - demo_marker_publisher"
echo "  - demo_pointcloud_publisher"
echo "  - demo_tf_broadcaster"
echo ""
echo "Published topics:"
echo "  - /demo_markers     (visualization_msgs/MarkerArray)"
echo "  - /demo_pointcloud  (sensor_msgs/PointCloud2)"
echo ""
echo "TF frames:"
echo "  map -> odom -> base_link -> laser_frame, camera_frame, arm_*"
echo ""

# Run all demo nodes
ros2 run wiz_example demo_marker_publisher &
ros2 run wiz_example demo_pointcloud_publisher &
ros2 run wiz_example demo_tf_broadcaster &

# Wait for any process to exit
wait -n

# Exit with status of process that exited first
exit $?
