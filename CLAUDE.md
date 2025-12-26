# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

wiz is a ROS2 visualization tool (Rviz alternative) built with Rust. It uses WebGPU for rendering and supports both native and WASM targets.

## Build Commands

```bash
# Check all targets
cargo check --all-targets

# Run tests
cargo test --all

# Build release
cargo build --release

# Format check
cargo fmt --all -- --check

# Lint
cargo clippy --all-targets -- -D warnings

# WASM check (renderer only)
cargo check -p wiz-renderer --target wasm32-unknown-unknown
```

### Running

```bash
# Start server (mock data mode, no ROS2 required)
cargo run -p wiz-server

# Start frontend (separate terminal)
cargo run -p wiz-frontend

# Build server with ROS2 support
source /opt/ros/humble/setup.bash
cargo build --release -p wiz-server --features ros2
```

### Docker

```bash
# TurtleBot3 Gazebo simulation
xhost +local:docker
./scripts/run-ros2-example.sh

# Build images
docker compose build
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Frontend (wiz-frontend)         Backend (wiz-server)           │
│  ┌───────────────────────┐       ┌───────────────────────────┐  │
│  │ egui + egui_dock UI   │  WS   │ axum WebSocket server     │  │
│  │ wgpu/WebGPU renderer  │◄─────►│ MessagePack serialization │  │
│  │ Native or WASM        │       │ Mock data or ROS2 bridge  │  │
│  └───────────────────────┘       └─────────────┬─────────────┘  │
│                                                │ cxx FFI        │
│                                  ┌─────────────▼─────────────┐  │
│                                  │ ROS2 Bridge (C++/rclcpp)  │  │
│                                  └───────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Crate Responsibilities

- **wiz-core**: Common types, transform math, ROS2-equivalent message structs
- **wiz-renderer**: WebGPU rendering engine (camera, pipelines, shaders)
- **wiz-frontend**: egui application (panels, state management, WebSocket client)
- **wiz-server**: axum WebSocket server, mock data generator, ROS2 bridge FFI
- **wiz-protocol**: MessagePack message types for client-server communication

## Key Technical Details

- **Rust Edition**: 2024, toolchain 1.88.0
- **Graphics**: wgpu 23.0, egui 0.30, eframe 0.30
- **Async**: tokio + axum for server, ewebsock for cross-platform WebSocket client
- **FFI**: cxx for type-safe Rust/C++ interop with ROS2
- **Serialization**: MessagePack (rmp-serde) over WebSocket

## Development Workflow

See `specs/SPECIFICATION.md` for TODO items organized by development phase:
- Phase 1: Foundation (MVP) - completed
- Phase 2: Core Features - in progress
- Phase 3: Extended features
- Phase 4: Polish

Use the `/wiz-dev` skill to work through spec TODOs and update `docs/` after completing tasks.

## Commit Convention

Uses commitlint with conventional commits. Pre-commit hooks via husky enforce formatting.
