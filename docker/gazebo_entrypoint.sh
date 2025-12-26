#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Ensure Gazebo can find ROS2 plugins
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH}

echo "Starting TurtleBot3 Gazebo simulation..."
echo "Model: ${TURTLEBOT3_MODEL:-waffle_pi}"

# Start Gazebo server and client in background
ros2 launch gazebo_ros gzserver.launch.py world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world &
GZSERVER_PID=$!

ros2 launch gazebo_ros gzclient.launch.py &
GZCLIENT_PID=$!

# Start robot_state_publisher
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true &

# Wait for spawn_entity service with extended timeout (120s)
echo "Waiting for Gazebo spawn_entity service (may take 60-90 seconds)..."
TIMEOUT=120
ELAPSED=0
while ! ros2 service list 2>/dev/null | grep -q /spawn_entity; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))
    echo "  Waiting... (${ELAPSED}s / ${TIMEOUT}s)"
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "ERROR: spawn_entity service not available after ${TIMEOUT}s"
        exit 1
    fi
done
echo "Gazebo spawn_entity service is ready!"

# Spawn TurtleBot3
echo "Spawning TurtleBot3 ${TURTLEBOT3_MODEL:-waffle_pi}..."
ros2 run gazebo_ros spawn_entity.py \
    -entity ${TURTLEBOT3_MODEL:-waffle_pi} \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL:-waffle_pi}/model.sdf \
    -x -2.0 -y -0.5 -z 0.01

echo "TurtleBot3 spawned successfully!"
echo "Available topics:"
ros2 topic list

# Keep container running
wait $GZSERVER_PID
