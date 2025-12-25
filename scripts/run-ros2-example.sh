#!/bin/bash
# Run wiz with ROS2 example using Docker
#
# This script starts:
# 1. ROS2 example publisher (publishes sample sensor data)
# 2. wiz-server with ROS2 bridge (connects to ROS2 network)
#
# Then you can run wiz-frontend locally to visualize the data.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "============================================="
echo "  wiz - ROS2 Visualization Tool"
echo "  Docker Example Mode"
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
$COMPOSE_CMD build ros2-example wiz-server-humble

echo ""
echo "Starting ROS2 example publisher and wiz-server..."
echo ""
echo "The following services will start:"
echo "  - ros2-example:     Publishes sample LaserScan and PointCloud2"
echo "  - wiz-server:       WebSocket server with ROS2 bridge"
echo ""
echo "After services start, run the frontend locally:"
echo "  cargo run -p wiz-frontend"
echo ""
echo "Or access via browser if using WASM build."
echo ""
echo "Press Ctrl+C to stop all services."
echo ""

# Start services
$COMPOSE_CMD --profile example up ros2-example wiz-server-humble
