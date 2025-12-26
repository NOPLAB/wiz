# wiz ROS2 Package

ROS2 package for wiz - a WebGPU-based ROS2 visualization tool.

## Overview

This package provides ROS2 launch files for wiz, allowing it to be started from standard ROS2 launch files and integrated with other ROS2 packages.

## Prerequisites

- ROS2 Humble or later
- Rust (cargo)

## Building

```bash
# Navigate to wiz project root
cd /path/to/wiz

# Build with colcon
colcon build --packages-select wiz

# Source the workspace
source install/setup.bash
```

## Usage

### Launch wiz (server + frontend)

```bash
ros2 launch wiz wiz.launch.py
```

### Launch server only

```bash
ros2 launch wiz server.launch.py
```

### Launch frontend only (connect to existing server)

```bash
ros2 launch wiz frontend.launch.py server_url:=ws://192.168.1.100:9090/ws
```

## Launch Arguments

### wiz.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `port` | `9090` | WebSocket server port |
| `host` | `0.0.0.0` | WebSocket server host address |
| `server_url` | `ws://localhost:9090/ws` | Server URL for frontend |

### server.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `port` | `9090` | WebSocket server port |
| `host` | `0.0.0.0` | WebSocket server host address |
| `log_level` | `info` | Log level (trace, debug, info, warn, error) |

### frontend.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `server_url` | `ws://localhost:9090/ws` | WebSocket server URL to connect |
| `log_level` | `info` | Log level (trace, debug, info, warn, error) |

## Integration with Other Packages

Include wiz in your launch file:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Your nodes here...

        # Include wiz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('wiz'), '/launch/wiz.launch.py'
            ]),
            launch_arguments={
                'port': '9090',
            }.items(),
        ),
    ])
```

## Supported Message Types

- `sensor_msgs/msg/PointCloud2`
- `sensor_msgs/msg/LaserScan`
- `geometry_msgs/msg/PoseStamped`
- `visualization_msgs/msg/MarkerArray`
- TF2 transforms

## License

MIT
