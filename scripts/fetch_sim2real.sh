#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/.cache/sim2real"
PIN=53324d40def0dd753b4a99021b8fe596955a1ecd
if [ ! -d "$DEST/.git" ]; then
  git -c http.proxy= clone --filter=blob:none --no-checkout https://github.com/p20030920p/Sim2Real-AlgoBench.git "$DEST"
  git -C "$DEST" config http.proxy ''
  git -C "$DEST" sparse-checkout set src/race_gazebo/models/competition_map_dae
  git -C "$DEST" checkout --detach "$PIN"
fi
test "$(git -C "$DEST" rev-parse HEAD)" = "$PIN"
test -z "$(git -C "$DEST" status --porcelain --untracked-files=no)"
echo "SIM2REAL_ASSETS_READY $PIN"
