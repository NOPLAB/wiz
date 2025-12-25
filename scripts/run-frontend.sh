#!/bin/bash
# Run wiz frontend only
#
# Use this when the server is already running.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "Starting wiz-frontend..."
echo "Make sure wiz-server is running on localhost:9090"
echo ""

cargo run --release -p wiz-frontend
