#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME="${SLAM_AUTHOR_RUNTIME:-/home/qzl/projects/SLAM_Author_Originals}"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
exec "$RUNTIME/envs/lidar/bin/python" -u -m slam_physical.localization_maps --runtime "$RUNTIME" "$@"
