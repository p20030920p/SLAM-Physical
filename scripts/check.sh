#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
for script in "$ROOT"/scripts/*.sh; do bash -n "$script"; done
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
"${SLAM_AUTHOR_RUNTIME:-/home/qzl/projects/SLAM_Author_Originals}/envs/lidar/bin/python" -m unittest discover -s "$ROOT/tests" -v
