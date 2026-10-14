"""Run author mappers on one training drive, then export AMCL occupancy maps."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from slam_physical.execution import check_stop, run_logged
from slam_physical.geometry import world_points
from slam_physical.native import run_beautymap
from slam_physical.occupancy import project_cloud, save_grid
from slam_physical.storage import digest, read_pcd, write_json

__all__ = ["build_maps", "main"]


def rgbd_geometry(source: Path, maximum: int):
    from PIL import Image

    root = source / "rgbd"
    camera = json.loads((root / "cam_params.json").read_text())["camera"]
    paths = sorted((root / "room0/results").glob("depth*.png"))
    poses = np.loadtxt(root / "room0/traj.txt", ndmin=2).reshape(-1, 4, 4)
    selected = np.linspace(0, len(paths) - 1, min(maximum, len(paths)), dtype=int)
    clouds, origins = [], []
    for index in selected:
        depth = np.array(Image.open(paths[index]), dtype=float) / camera["scale"]
        yy, xx = np.indices(depth.shape)
        valid = (depth > 0.15) & (depth < 11.99)
        xyz = np.column_stack(
            (
                (xx[valid] - camera["cx"]) * depth[valid] / camera["fx"],
                (yy[valid] - camera["cy"]) * depth[valid] / camera["fy"],
                depth[valid],
            )
        )
        points = xyz @ poses[index, :3, :3].T + poses[index, :3, 3]
        clouds.append(points.astype(np.float32))
        origins.append(np.broadcast_to(poses[index, :3, 3], points.shape).copy())
    return np.concatenate(clouds), np.concatenate(origins), selected.tolist()


def build_maps(
    source: Path,
    output: Path,
    runtime: Path,
    frames=40,
    semantic_frames=8,
    methods=("raw", "beautymap", "dufomap", "conceptgraphs", "hovsg"),
):
    if not (source / "mapping_scans.npz").is_file():
        raise ValueError("Finish a keyboard mapping drive first")
    output.mkdir(parents=True, exist_ok=False)
    archive = np.load(source / "mapping_scans.npz", allow_pickle=False)
    selected = np.linspace(
        0, len(archive["poses"]) - 1, min(frames, len(archive["poses"])), dtype=int
    )
    scans = [archive[f"scan_{i:04d}"] for i in selected]
    poses = archive["poses"][selected]
    raw, frame_ids = world_points(scans, poses)
    origins = poses[frame_ids, :3, 3]
    manifest = {
        "mapping_recording": str(source.resolve()),
        "source_scans_sha256": digest(source / "mapping_scans.npz"),
        "pose_source": "wheel odometry with declared start anchor and fixed extrinsics; no truth poses",
        "lidar_observation_ids": selected.tolist(),
        "semantic_frames_requested": semantic_frames,
        "methods": {},
        "scope": "offline mapping followed by common AMCL; semantic geometry projection does not use CLIP or hierarchy during localization",
    }
    maps = output / "maps"
    maps.mkdir()
    for method in methods:
        check_stop(output)
        print("BUILD MAP: " + method, flush=True)
        started = time.perf_counter()
        try:
            evidence, evidence_origins = raw, origins
            family = "lidar"
            if method == "raw":
                points = raw
            elif method == "beautymap":
                mask, record = run_beautymap(
                    scans,
                    poses,
                    output / "beautymap-native",
                    runtime / "upstream/beautymap",
                    {"dis_range": 8, "xy_resolution": 0.5, "h_res": 0.2},
                )
                points = raw[mask]
            elif method == "dufomap":
                from dufomap import dufomap

                mapper = dufomap(0.1, 0.2, 1, num_threads=2)
                for index, (scan, pose) in enumerate(zip(scans, poses)):
                    check_stop(output)
                    cloud = scan @ pose[:3, :3].T + pose[:3, 3]
                    # cloud_transform=False: input XYZ is already in the map frame.
                    viewpoint = [*map(float, pose[:3, 3]), 1.0, 0.0, 0.0, 0.0]
                    mapper.run(cloud, viewpoint, cloud_transform=False)
                    print(f"DUFOMap mapping frame {index+1}/{len(scans)}", flush=True)
                mapper.oncePropagateCluster(if_propagate=True, if_cluster=False)
                import os

                native = output / "dufomap-native"
                native.mkdir()
                previous = Path.cwd()
                try:
                    os.chdir(native)
                    mapper.outputMap(raw, voxel_map=False)
                finally:
                    os.chdir(previous)
                points = read_pcd(native / "dufomap_output.pcd")
            else:
                family = "rgbd_geometry_projection"
                evidence, evidence_origins, ids = rgbd_geometry(source, semantic_frames)
                if "rgbd-raw" not in manifest["methods"]:
                    save_grid(
                        maps / "rgbd-raw",
                        project_cloud(evidence, evidence, evidence_origins),
                        {
                            "method": "rgbd-raw",
                            "mapping_recording": str(source),
                            "rgbd_ids": ids,
                        },
                    )
                    manifest["methods"]["rgbd-raw"] = {
                        "status": "completed",
                        "family": family,
                        "source_frames": ids,
                        "map_yaml": str(maps / "rgbd-raw/map.yaml"),
                    }
                native = output / (method + "-native")
                native.mkdir()
                command = [
                    str(runtime / "envs" / method / "bin/python"),
                    "-u",
                    str(Path(__file__).with_name("semantic_worker.py")),
                    "--method",
                    method,
                    "--runtime",
                    str(runtime),
                    "--output",
                    str(native),
                    "--frames",
                    str(semantic_frames),
                    "--recording",
                    str(source / "rgbd"),
                ]
                code = run_logged(command, native, native / "native.log", output)
                if code:
                    raise RuntimeError(
                        "Native semantic pipeline failed; see "
                        + str(native / "native.log")
                    )
                points = np.load(native / "actual_map.npy", allow_pickle=False)
            projection_started = time.perf_counter()
            map_yaml = save_grid(
                maps / method,
                project_cloud(points, evidence, evidence_origins),
                {
                    "method": method,
                    "mapping_recording": str(source),
                    "family": family,
                    "output_points": len(points),
                    "free_space": "common raw sensor rays within family; no SDF truth fill",
                },
            )
            manifest["methods"][method] = {
                "status": "completed",
                "family": family,
                "output_points": len(points),
                "map_yaml": str(map_yaml),
                "mapping_and_export_wall_s": time.perf_counter() - started,
                "projection_wall_s": time.perf_counter() - projection_started,
            }
            print("MAP READY: " + str(map_yaml), flush=True)
        except InterruptedError:
            raise
        except Exception as error:
            manifest["methods"][method] = {"status": "failed", "error": str(error)}
            print("MAP FAILED: " + method + ": " + str(error), flush=True)
        write_json(output / "maps.json", manifest)
    if not any(item["status"] == "completed" for item in manifest["methods"].values()):
        raise RuntimeError("No map was built")
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--semantic-frames", type=int, default=8)
    parser.add_argument(
        "--methods",
        nargs="+",
        choices=["raw", "beautymap", "dufomap", "conceptgraphs", "hovsg"],
        default=["raw", "beautymap", "dufomap", "conceptgraphs", "hovsg"],
    )
    args = parser.parse_args()
    build_maps(
        args.source.resolve(),
        args.output.resolve(),
        args.runtime.resolve(),
        args.frames,
        args.semantic_frames,
        args.methods,
    )


if __name__ == "__main__":
    main()
