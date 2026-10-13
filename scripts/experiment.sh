#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME="${SLAM_AUTHOR_RUNTIME:-/home/qzl/projects/SLAM_Author_Originals}"
PYTHON_BIN="${SLAM_BEAUTYMAP_PYTHON:-$RUNTIME/envs/lidar/bin/python}"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
exec "$PYTHON_BIN" -u -m slam_physical.cli experiment --upstream "$RUNTIME/upstream/beautymap" "$@"
