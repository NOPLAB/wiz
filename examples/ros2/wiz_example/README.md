# wiz_example

Example ROS2 package demonstrating how to use wiz with other ROS2 nodes.

## Overview

This package provides demo nodes that publish various ROS2 message types for testing wiz visualization capabilities:

- **demo_marker_publisher**: Publishes animated `MarkerArray` messages
- **demo_pointcloud_publisher**: Publishes animated `PointCloud2` messages
- **demo_tf_broadcaster**: Broadcasts animated TF frames

## Building

```bash
# Navigate to wiz project root
cd /path/to/wiz

# Build wiz and wiz_example
colcon build --packages-select wiz wiz_example

# Source the workspace
source install/setup.bash
```

## Usage

### Full Demo (wiz + all demo nodes)

```bash
ros2 launch wiz_example demo_with_wiz.launch.py
```

This launches:
- wiz-server (WebSocket server)
- wiz-frontend (GUI)
- demo_marker_publisher
- demo_pointcloud_publisher
- demo_tf_broadcaster

### Individual Demo Nodes

```bash
# First, start wiz
ros2 launch wiz wiz.launch.py

# Then in separate terminals, run demo nodes:
ros2 run wiz_example demo_marker_publisher
ros2 run wiz_example demo_pointcloud_publisher
ros2 run wiz_example demo_tf_broadcaster
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/demo_markers` | `visualization_msgs/MarkerArray` | Animated shapes (cube, sphere, cylinder, arrow) |
| `/demo_pointcloud` | `sensor_msgs/PointCloud2` | Animated rainbow spiral point cloud |

## TF Frames

The demo TF broadcaster publishes the following frame hierarchy:

```
map
└── odom
    └── base_link (moving in figure-8 pattern)
        ├── laser_frame
        ├── camera_frame
        └── arm_base
            └── arm_link1 (rotating)
                └── arm_link2 (rotating)
                    └── end_effector
```

## License

MIT
