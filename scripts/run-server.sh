#!/bin/bash
# Run wiz server only
#
# Starts the WebSocket server with mock data generation.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "Starting wiz-server on port 9090..."
echo ""
echo "Available endpoints:"
echo "  - WebSocket: ws://localhost:9090/ws"
echo "  - Health:    http://localhost:9090/health"
echo ""

# Optional: Set log level
export RUST_LOG="${RUST_LOG:-wiz_server=info}"

cargo run --release -p wiz-server
