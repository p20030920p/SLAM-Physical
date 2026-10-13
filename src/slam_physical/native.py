"""Execute a pinned author checkout; instrumentation only in a per-run copy."""

from __future__ import annotations

import difflib
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from slam_physical.execution import run_logged
from slam_physical.storage import digest, read_pcd, write_json, write_pcd

BEAUTYMAP_COMMIT = "98bce4a97db96ddd0d5342e31425c7679f58ba2e"
__all__ = ["BEAUTYMAP_COMMIT", "run_beautymap"]


def run_beautymap(
    scans: list[np.ndarray],
    poses: np.ndarray,
    output: Path,
    upstream: Path,
    parameters: dict,
) -> tuple[np.ndarray, dict]:
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=upstream, text=True
    ).strip()
    status = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=upstream,
        text=True,
    )
    if commit != BEAUTYMAP_COMMIT or status.strip():
        raise RuntimeError(
            "BeautyMap checkout must be clean and match dependencies.lock.json"
        )
    output.mkdir(parents=True, exist_ok=False)
    work = output / "author-code"
    shutil.copytree(
        upstream,
        work,
        ignore=shutil.ignore_patterns(".git", "__pycache__", "data", "assets"),
    )
    patches = []
    reader = work / "utils/pcdpy3.py"
    original = reader.read_text()
    changed = original.replace(
        "metadata[key] = map(int, value.split())",
        "metadata[key] = list(map(int, value.split()))",
    )
    changed = changed.replace(
        "metadata[key] = map(float, value.split())",
        "metadata[key] = list(map(float, value.split()))",
    )
    if changed != original:
        reader.write_text(changed)
        patches.extend(
            difflib.unified_diff(
                original.splitlines(True),
                changed.splitlines(True),
                fromfile="a/utils/pcdpy3.py",
                tofile="b/utils/pcdpy3.py",
            )
        )
    entry = work / "main.py"
    original = entry.read_text()
    anchor = 'save_pcd(f"{data_dir}/beautymap_output.pcd", Mpts.original_points[points_index2Retain])'
    if original.count(anchor) != 1:
        raise RuntimeError("Unexpected native export code")
    changed = original.replace(
        anchor,
        'np.save(f"{data_dir}/retained_indices.npy", points_index2Retain)\n    '
        + anchor,
    )
    entry.write_text(changed)
    patches.extend(
        difflib.unified_diff(
            original.splitlines(True),
            changed.splitlines(True),
            fromfile="a/main.py",
            tofile="b/main.py",
        )
    )
    (output / "instrumentation.patch").write_text("".join(patches))
    data = output / "input"
    (data / "pcd").mkdir(parents=True)
    all_points = []
    input_hashes = {}
    for i, (scan, pose) in enumerate(zip(scans, poses)):
        points = scan @ pose[:3, :3].T + pose[:3, 3]
        all_points.append(points)
        path = data / "pcd" / f"{i:06d}.pcd"
        write_pcd(path, points, pose[:3, 3])
        input_hashes[path.name] = digest(path)
    write_pcd(data / "raw_map.pcd", np.concatenate(all_points))
    command = [sys.executable, "-u", str(entry), "--data_dir", str(data)]
    for key, value in parameters.items():
        command.extend([f"--{key}", str(value)])
    started = time.perf_counter()
    print("NATIVE BEAUTYMAP:", " ".join(command), flush=True)
    code = run_logged(command, work, output / "native.log", output)
    record = {
        "command": command,
        "commit": commit,
        "exit_code": code,
        "seconds": time.perf_counter() - started,
        "frames": len(scans),
        "parameters": parameters,
        "scan_sha256": input_hashes,
        "raw_map_sha256": digest(data / "raw_map.pcd"),
        "instrumentation_sha256": digest(output / "instrumentation.patch"),
        "scope": "Original classification code; iterator compatibility and retained-index export only",
    }
    write_json(output / "record.json", record)
    if code:
        raise RuntimeError(f"Native BeautyMap failed; see {output / 'native.log'}")
    kept = np.load(data / "retained_indices.npy", allow_pickle=False)
    if (
        len(np.unique(kept)) != len(kept)
        or np.any(kept < 0)
        or np.any(kept >= sum(map(len, scans)))
    ):
        raise RuntimeError("Invalid native source-point indices")
    cloud = read_pcd(data / "beautymap_output.pcd")
    expected = np.concatenate(all_points).astype(np.float32)[kept]
    if cloud.shape != expected.shape or not np.allclose(
        cloud, expected, atol=2e-5, rtol=0
    ):
        raise RuntimeError("Native point-index export does not match native PCD")
    mask = np.zeros(sum(map(len, scans)), dtype=bool)
    mask[kept] = True
    np.save(output / "keep_mask.npy", mask)
    return mask, record
