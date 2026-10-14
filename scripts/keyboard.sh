#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
RUN="$1"
export ROS_DOMAIN_ID="$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["ros_domain_id"])' "$RUN/session.json")"
echo 'Hold i to move forward, j/l to rotate, k to stop. q/z change speed. Ctrl+C finishes this session.'
exec python3 -m slam_physical.keyboard_session "$RUN"
