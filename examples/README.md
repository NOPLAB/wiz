# wiz Examples

This directory contains examples and scripts for testing wiz.

## Quick Start

### Option 1: Mock Data (No ROS2 Required)

The server includes a built-in mock data generator that produces simulated sensor data.

1. **Start the server:**
   ```bash
   ./scripts/run-demo.sh
   ```
   Or manually:
   ```bash
   cargo run -p wiz-server
   ```

2. **Start the frontend (in another terminal):**
   ```bash
   cargo run -p wiz-frontend
   ```

3. **Connect and subscribe:**
   - The frontend will show a connection dialog
   - Connect to `ws://localhost:9090/ws`
   - Use the Topics panel to subscribe to topics
   - Watch the 3D viewport for visualized data

### Option 2: Docker with ROS2 (Full ROS2 Integration)

Use Docker to run a complete ROS2 environment with sample publishers.

1. **Start the Docker services:**
   ```bash
   ./scripts/run-ros2-example.sh
   ```

   This will start:
   - `ros2-example`: ROS2 node publishing sample sensor data
   - `wiz-server-humble`: wiz server with ROS2 bridge

2. **Start the frontend locally:**
   ```bash
   cargo run -p wiz-frontend
   ```

3. **Connect and visualize:**
   - Connect to `ws://localhost:9090/ws`
   - Subscribe to `/scan` or `/velodyne_points`

### Option 3: Connect to Your ROS2 System

If you have ROS2 running on your system:

1. **Build wiz-server with ROS2 support:**
   ```bash
   source /opt/ros/humble/setup.bash
   cargo build --release -p wiz-server --features ros2
   ```

2. **Run the server:**
   ```bash
   ./target/release/wiz-server
   ```

3. **Start the frontend and connect.**

## Available Topics

### Mock Data Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/velodyne_points` | PointCloud2 | Animated 3D spiral point cloud |
| `/ground_plane` | PointCloud2 | Checkerboard ground plane |
| `/scan` | LaserScan | 360-degree laser scan with moving obstacles |
| `/scan_front` | LaserScan | Front-facing laser scan |

### ROS2 Example Topics

When using the Docker ROS2 example (`ros2-example`):

| Topic | Type | Description |
|-------|------|-------------|
| `/velodyne_points` | PointCloud2 | Spiral point cloud |
| `/scan` | LaserScan | 360-degree scan with obstacles |

## Files

- `ros2_publisher.py` - Python ROS2 node that publishes sample sensor data
- `../scripts/run-demo.sh` - Run server with mock data
- `../scripts/run-ros2-example.sh` - Run Docker ROS2 example
- `../scripts/run-server.sh` - Run server only
- `../scripts/run-frontend.sh` - Run frontend only

## Docker Commands

Build images:
```bash
docker-compose build
```

Run ROS2 example:
```bash
docker-compose --profile example up ros2-example wiz-server-humble
```

Run only wiz-server (when using external ROS2):
```bash
docker-compose up wiz-server-humble
```

## Troubleshooting

### Connection refused
- Make sure the server is running on port 9090
- Check if another process is using the port: `lsof -i :9090`

### No data received
- Ensure you've subscribed to a topic in the Topics panel
- Check server logs for subscription confirmations

### Docker networking issues
- Use `--net=host` for direct ROS2 DDS communication
- Set `ROS_DOMAIN_ID` environment variable if needed
