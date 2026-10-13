#!/usr/bin/env bash
set -euo pipefail
if [ "$(id -u)" != 0 ]; then
  exec sudo bash "$0"
fi
. /etc/os-release
test "$VERSION_ID" = 24.04 || { echo 'Use the dedicated Ubuntu-24.04 distro.'; exit 1; }
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y ca-certificates curl gnupg locales git python3-numpy python3-scipy
locale-gen en_US.UTF-8
curl --fail --retry 3 --noproxy '*' -L https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main\n' "$(dpkg --print-architecture)" > /etc/apt/sources.list.d/ros2.list
apt-get update
apt-get install -y ros-jazzy-ros-base ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
echo 'SIMULATION_READY: Ubuntu 24.04 / ROS Jazzy / Gazebo Harmonic'
