#!/bin/bash
# Run wiz with TurtleBot3 Gazebo simulation using Docker
#
# This script starts:
# 1. TurtleBot3 Gazebo simulation (publishes realistic sensor data)
# 2. wiz-server with ROS2 bridge (connects to ROS2 network)
#
# Then you can run wiz-frontend locally to visualize the data.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "============================================="
echo "  wiz - ROS2 Visualization Tool"
echo "  TurtleBot3 Gazebo Simulation Mode"
echo "============================================="
echo ""

# Check Docker
if ! command -v docker &> /dev/null; then
    echo "Error: docker is not installed."
    exit 1
fi

if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: docker-compose is not installed."
    exit 1
fi

# Determine docker compose command
if docker compose version &> /dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

# X11 setup for Gazebo GUI
echo "Setting up X11 forwarding for Gazebo..."
if command -v xhost &> /dev/null; then
    xhost +local:docker 2>/dev/null || true
else
    echo "Warning: xhost not found. Gazebo GUI may not display."
    echo "Install with: sudo apt install x11-xserver-utils"
fi

echo ""
echo "Building Docker images (this may take a while for Gazebo)..."
$COMPOSE_CMD build gazebo-turtlebot3 wiz-server-humble

echo ""
echo "Starting TurtleBot3 Gazebo simulation and wiz-server..."
echo ""
echo "The following services will start:"
echo "  - gazebo-turtlebot3: TurtleBot3 (waffle_pi) in Gazebo world"
echo "  - wiz-server:        WebSocket server with ROS2 bridge"
echo ""
echo "Published topics:"
echo "  - /scan              (sensor_msgs/LaserScan)"
echo "  - /odom              (nav_msgs/Odometry)"
echo "  - /tf                (tf2_msgs/TFMessage)"
echo "  - /camera/depth/points (sensor_msgs/PointCloud2)"
echo ""
echo "After services start, run the frontend locally:"
echo "  cargo run -p wiz-frontend"
echo ""
echo "To control the robot (in another terminal):"
echo "  docker exec -it \$(docker ps -qf name=gazebo) ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "Press Ctrl+C to stop all services."
echo ""

# Start services
$COMPOSE_CMD --profile example up gazebo-turtlebot3 wiz-server-humble
