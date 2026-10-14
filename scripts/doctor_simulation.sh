#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
command -v gz
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix nav2_amcl
ros2 pkg prefix nav2_map_server
ros2 pkg prefix teleop_twist_keyboard
command -v rviz2
python3 -c 'import rclpy,numpy,scipy; print("SIMULATION_READY")'
