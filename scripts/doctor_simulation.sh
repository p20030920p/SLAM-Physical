#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
command -v gz
ros2 pkg prefix ros_gz_bridge
python3 -c 'import rclpy,numpy,scipy; print("SIMULATION_READY")'
