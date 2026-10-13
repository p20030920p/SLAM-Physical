#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNTIME="${SLAM_AUTHOR_RUNTIME:-/home/qzl/projects/SLAM_Author_Originals}"
set +u
source /opt/ros/humble/setup.bash
set -u
command -v rviz2
python3 -c 'import rclpy,numpy; print("VIEWER_READY")'
"$RUNTIME/envs/lidar/bin/python" -c 'import numpy,scipy,fire,dztimer; print("BEAUTYMAP_READY")'
"$RUNTIME/envs/lidar/bin/python" -c 'import dufomap; print("DUFOMAP_READY")'
"$RUNTIME/envs/conceptgraphs/bin/python" -c 'import torch,open3d,segment_anything; print("CONCEPTGRAPHS_READY", torch.cuda.is_available())'
"$RUNTIME/envs/hovsg/bin/python" -c 'import torch,open3d,open_clip; print("HOVSG_READY", torch.cuda.is_available())'
test "$(git -C "$RUNTIME/upstream/beautymap" rev-parse HEAD)" = 98bce4a97db96ddd0d5342e31425c7679f58ba2e
for method in beautymap conceptgraphs hovsg; do
  test -z "$(git -C "$RUNTIME/upstream/$method" status --porcelain --untracked-files=no)"
done
test -f "$RUNTIME/weights/sam_vit_h_4b8939.pth"
test -f "$RUNTIME/weights/laion2b_s32b_b79k.bin"
test -f "$ROOT/.cache/sim2real/src/race_gazebo/models/competition_map_dae/model.sdf"
