"""Fresh native executions on declared demo subsets; never play saved videos."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np

from slam_physical.execution import check_stop, run_logged
from slam_physical.native import run_beautymap
from slam_physical.storage import digest, publish_cloud, read_pcd, write_json

__all__ = ["reproduce"]


def _viewpoint(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        for line in stream:
            if line.startswith(b"VIEWPOINT "):
                return np.array([float(v) for v in line.split()[1:]])
            if line.startswith(b"DATA "):
                break
    raise ValueError("PCD viewpoint missing")


def reproduce(method: str, output: Path, runtime: Path, frames: int = 8) -> None:
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    if method in ("beautymap", "dufomap"):
        paths = sorted((runtime / "data/00-pristine/pcd").glob("*.pcd"))
        if frames:
            paths = paths[:frames]
        if not paths:
            raise FileNotFoundError(
                "Prepare released KITTI-00 scans in the existing author runtime"
            )
        global_points = [read_pcd(p) for p in paths]
        viewpoints = [_viewpoint(p) for p in paths]
        poses = np.repeat(np.eye(4)[None], len(paths), axis=0)
        poses[:, :3, 3] = [v[:3] for v in viewpoints]
        # Released scans are already in world coordinates. Use local XYZ with
        # identity orientation so native staging preserves the exact world XYZ.
        scans = [p - v[:3] for p, v in zip(global_points, viewpoints)]
        publish_cloud(
            output, "raw", np.concatenate(global_points), "Fresh KITTI native execution"
        )
        if method == "beautymap":
            mask, record = run_beautymap(
                scans,
                poses,
                output / "native",
                runtime / "upstream/beautymap",
                {"dis_range": 40, "xy_resolution": 1.0, "h_res": 0.5},
            )
            points = np.concatenate(global_points)[mask]
        else:
            from dufomap import dufomap

            work = output / "native"
            work.mkdir()
            previous = Path.cwd()
            os.chdir(work)
            try:
                mapper = dufomap(0.1, 0.2, 1, num_threads=2)
                for i, (points, viewpoint) in enumerate(zip(global_points, viewpoints)):
                    check_stop(output)
                    radius = np.linalg.norm(points - viewpoint[:3], axis=1)
                    mapper.run(
                        points[(radius > 0.2) & (radius < 50)],
                        viewpoint.tolist(),
                        cloud_transform=False,
                    )
                    print(f"DUFOMap actual frame {i+1}/{len(paths)}", flush=True)
                mapper.oncePropagateCluster(if_propagate=True, if_cluster=False)
                mapper.outputMap(np.concatenate(global_points), voxel_map=False)
                points = read_pcd(work / "dufomap_output.pcd")
            finally:
                os.chdir(previous)
            record = {
                "parameters": {"voxel": 0.1, "d_s": 0.2, "d_p": 1},
                "frames": len(paths),
            }
        scope = "Fresh native mapping demo on released KITTI scans; accumulated raw map, not paper-table scoring"
        write_json(
            output / "input.json",
            {"files": [{"name": p.name, "sha256": digest(p)} for p in paths]},
        )
    else:
        environment = "conceptgraphs" if method == "conceptgraphs" else "hovsg"
        python = runtime / "envs" / environment / "bin/python"
        command = [
            str(python),
            "-u",
            str(Path(__file__).with_name("semantic_worker.py")),
            "--method",
            method,
            "--runtime",
            str(runtime),
            "--output",
            str(output),
            "--frames",
            str(frames or 8),
        ]
        print("FRESH SEMANTIC PIPELINE:", " ".join(command), flush=True)
        code = run_logged(command, output, output / "native.log", output)
        if code:
            raise RuntimeError("Semantic pipeline failed; see native.log")
        points = np.load(output / "actual_map.npy", allow_pickle=False)
        record = json.loads((output / "semantic-record.json").read_text())
        scope = "Fresh SAM/CLIP and native mapping on resized Replica demo subset; no semantic benchmark, hierarchy or navigation claim"
    publish_cloud(output, "C4", points, method + " native pipeline completed")
    write_json(
        output / "reproduction.json",
        {
            "method": method,
            "status": "completed",
            "scope": scope,
            "elapsed_seconds": time.perf_counter() - started,
            "record": record,
            "output_points": len(points),
        },
    )
