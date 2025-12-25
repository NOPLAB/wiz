#!/bin/bash
# Run wiz demo with mock data
#
# This script starts both the server and frontend for demonstration.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "==================================="
echo "  wiz - ROS2 Visualization Tool"
echo "  Demo Mode (Mock Data)"
echo "==================================="
echo ""

# Check if cargo is available
if ! command -v cargo &> /dev/null; then
    echo "Error: cargo is not installed. Please install Rust first."
    exit 1
fi

# Build in release mode for better performance
echo "Building wiz..."
cargo build --release -p wiz-server -p wiz-frontend 2>&1 | tail -5

echo ""
echo "Starting wiz-server on port 9090..."
echo "Mock data will be generated when you subscribe to topics."
echo ""
echo "Available topics:"
echo "  - /velodyne_points  (PointCloud2 - animated spiral)"
echo "  - /ground_plane     (PointCloud2 - checkerboard)"
echo "  - /scan             (LaserScan - 360 degree)"
echo ""
echo "Press Ctrl+C to stop."
echo ""

# Run the server
cargo run --release -p wiz-server
