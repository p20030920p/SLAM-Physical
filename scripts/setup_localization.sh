#!/usr/bin/env bash
set -euo pipefail
test "$(id -u)" = 0
apt-get update
apt-get install -y ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins python3-pil python3-matplotlib python3-yaml
