#!/bin/bash
# Clean launch for rover_gazebo: kills stale processes from a previous
# run before starting, so orphaned nodes don't pile up between runs.
set -e

pkill -f "ros2 launch rover_gazebo" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f ros_gz_bridge 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
sleep 1

source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/../.."
source install/setup.bash

exec ros2 launch rover_gazebo gazebo.launch.py
