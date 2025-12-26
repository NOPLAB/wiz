#!/bin/bash
# Run wiz with wiz_example demo nodes using Docker
#
# This script starts:
# 1. wiz_example demo nodes (MarkerArray, PointCloud2, TF)
# 2. wiz-server with ROS2 bridge
#
# Then you can run wiz-frontend locally to visualize the data.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "============================================="
echo "  wiz - ROS2 Visualization Tool"
echo "  wiz_example Demo Mode (Docker)"
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

echo "Building Docker images..."
$COMPOSE_CMD build wiz-example wiz-server-humble

echo ""
echo "Starting wiz_example demo and wiz-server..."
echo ""
echo "The following services will start:"
echo "  - wiz-example:  Demo nodes (MarkerArray, PointCloud2, TF)"
echo "  - wiz-server:   WebSocket server with ROS2 bridge"
echo ""
echo "Published topics:"
echo "  - /demo_markers     (visualization_msgs/MarkerArray)"
echo "  - /demo_pointcloud  (sensor_msgs/PointCloud2)"
echo ""
echo "TF frames:"
echo "  map -> odom -> base_link -> laser_frame, camera_frame, arm_*"
echo ""
echo "After services start, run the frontend locally:"
echo "  cargo run -p wiz-frontend"
echo ""
echo "Press Ctrl+C to stop all services."
echo ""

# Start services
$COMPOSE_CMD --profile example up wiz-example wiz-server-humble
