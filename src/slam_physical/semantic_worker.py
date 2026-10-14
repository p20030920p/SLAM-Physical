"""Run in the method's existing environment; fresh front end and isolated inputs."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import pickle
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def prepare(runtime, output, frames, recording=None):
    original = recording if recording else runtime / "data/replica-full/Replica"
    target = output / "input"
    scene = target / "room0"
    (scene / "results").mkdir(parents=True)
    rgb = sorted((original / "room0/results").glob("frame*.jpg"))
    depth = sorted((original / "room0/results").glob("depth*.png"))
    if not rgb or len(rgb) != len(depth):
        raise RuntimeError("Replica RGB/depth inputs unavailable")
    selected = np.linspace(0, len(rgb) - 1, min(frames, len(rgb)), dtype=int)
    trajectory = (original / "room0/traj.txt").read_text().splitlines()
    lines = []
    hashes = []
    for i, source_id in enumerate(selected):
        rgb_image = Image.open(rgb[source_id])
        old_w, old_h = rgb_image.size
        rgb_image.resize((320, 180), Image.Resampling.BILINEAR).save(
            scene / "results" / f"frame{i:06d}.jpg"
        )
        Image.open(depth[source_id]).resize((320, 180), Image.Resampling.NEAREST).save(
            scene / "results" / f"depth{i:06d}.png"
        )
        lines.append(trajectory[source_id])
        hashes.append(
            {
                "source_id": int(source_id),
                "rgb": hashlib.sha256(rgb[source_id].read_bytes()).hexdigest(),
                "depth": hashlib.sha256(depth[source_id].read_bytes()).hexdigest(),
            }
        )
    (scene / "traj.txt").write_text("\n".join(lines) + "\n")
    camera = json.loads((original / "cam_params.json").read_text())
    parameters = camera["camera"]
    for name in ("fx", "cx"):
        parameters[name] *= 320 / old_w
    for name in ("fy", "cy"):
        parameters[name] *= 180 / old_h
    parameters["w"], parameters["h"] = 320, 180
    (target / "cam_params.json").write_text(json.dumps(camera, indent=2))
    return target, hashes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["conceptgraphs", "hovsg"], required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--recording", type=Path)
    args = parser.parse_args()
    runtime, output = args.runtime, args.output
    if not 2 <= args.frames <= 40:
        raise ValueError(
            "Demo accepts 2..40 frames; full benchmarks use the original protocol"
        )
    inputs, hashes = prepare(runtime, output, args.frames, args.recording)
    os.environ.update(
        {
            "OMP_NUM_THREADS": "2",
            "OPENBLAS_NUM_THREADS": "2",
            "MKL_NUM_THREADS": "2",
            "WANDB_MODE": "disabled",
            "HF_HUB_CACHE": str(runtime / "cache/huggingface/hub"),
            "GSA_PATH": str(runtime / "dependencies/Grounded-Segment-Anything"),
        }
    )
    method = args.method
    source = (
        runtime
        / "upstream"
        / ("conceptgraphs" if method == "conceptgraphs" else "hovsg")
    )
    pin = (
        "93277a02bd89171f8121e84203121cf7af9ebb5d"
        if method == "conceptgraphs"
        else "d6e65a53c8be6faec3f01f00d1644d967f89e605"
    )
    if (
        subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=source, text=True
        ).strip()
        != pin
    ):
        raise RuntimeError("Unexpected author source revision")
    if subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"], cwd=source, text=True
    ).strip():
        raise RuntimeError(
            "Author checkout has tracked changes; use an isolated clean checkout"
        )
    sys.path.insert(0, str(source))
    record = {
        "commit": pin,
        "source_frames": hashes,
        "resolution": [320, 180],
        "sam_batch": 16,
        "fresh_frontend": True,
        "input_kind": "recorded Gazebo RGB-D"
        if args.recording
        else "Replica demo subset",
    }
    if method == "hovsg":
        from hovsg.graph.graph import Graph
        from omegaconf import OmegaConf

        cfg = OmegaConf.load(source / "config/create_graph.yaml")
        cfg.main.dataset = "replica"
        cfg.main.dataset_path = str(inputs / "room0")
        cfg.main.package_path = str(source / "hovsg")
        cfg.main.save_path = str(output / "artifacts")
        cfg.main.scene_id = "room0"
        cfg.models.clip.checkpoint = str(runtime / "weights/laion2b_s32b_b79k.bin")
        cfg.models.sam.checkpoint = str(runtime / "weights/sam_vit_h_4b8939.pth")
        cfg.models.sam.points_per_batch = 16
        cfg.pipeline.skip_frames = 1
        cfg.pipeline.create_graph = False
        Path(cfg.main.save_path).mkdir()
        graph = Graph(cfg)
        graph.create_feature_map()
        graph.save_masked_pcds(path=cfg.main.save_path, state="both")
        graph.save_full_pcd(path=cfg.main.save_path)
        points = np.asarray(graph.full_pcd.points)
        record["config"] = OmegaConf.to_container(cfg, resolve=True)
    else:
        frontend = source / "conceptgraph/scripts/generate_gsa_results.py"
        code = frontend.read_text()
        if code.count("points_per_batch=144") != 1:
            raise RuntimeError("Unexpected SAM configuration")
        patched = output / "generate_gsa_results_batch16.py"
        patched.write_text(code.replace("points_per_batch=144", "points_per_batch=16"))
        env = os.environ.copy()
        env["PYTHONPATH"] = str(source) + os.pathsep + env.get("PYTHONPATH", "")
        config = source / "conceptgraph/dataset/dataconfigs/replica/replica.yaml"
        if args.recording:
            from omegaconf import OmegaConf

            dataset_config = OmegaConf.load(config)
            camera = json.loads((inputs / "cam_params.json").read_text())["camera"]
            for name, value in {
                "image_height": camera["h"],
                "image_width": camera["w"],
                "fx": camera["fx"],
                "fy": camera["fy"],
                "cx": camera["cx"],
                "cy": camera["cy"],
                "png_depth_scale": camera["scale"],
            }.items():
                dataset_config.camera_params[name] = value
            config = output / "recorded_replica.yaml"
            OmegaConf.save(dataset_config, config)
        front = [
            sys.executable,
            "-u",
            str(patched),
            "--dataset_root",
            str(inputs),
            "--dataset_config",
            str(config),
            "--scene_id",
            "room0",
            "--class_set",
            "none",
            "--stride",
            "1",
            "--desired_height",
            "180",
            "--desired_width",
            "320",
        ]
        subprocess.run(front, cwd=source / "conceptgraph", env=env, check=True)
        mapping = [
            sys.executable,
            "-u",
            str(source / "conceptgraph/slam/cfslam_pipeline_batch.py"),
            f"dataset_root={inputs}",
            f"dataset_config={config}",
            "scene_id=room0",
            "stride=1",
            "image_height=180",
            "image_width=320",
            "spatial_sim_type=overlap",
            "mask_conf_threshold=0.95",
            "match_method=sim_sum",
            "sim_threshold=1.2",
            "dbscan_eps=0.1",
            "gsa_variant=none",
            "skip_bg=True",
            "max_bbox_area_ratio=0.5",
            "save_suffix=live_demo",
            "save_objects_all_frames=True",
            "class_agnostic=True",
            "merge_interval=20",
            f"hydra.run.dir={output/'mapping-hydra'}",
        ]
        subprocess.run(mapping, cwd=source / "conceptgraph", env=env, check=True)
        native_map = inputs / "room0/pcd_saves/full_pcd_none_live_demo_post.pkl.gz"
        if not native_map.is_file():
            raise RuntimeError("Native ConceptGraphs produced no map")
        with gzip.open(native_map, "rb") as stream:
            data = pickle.load(stream)
        objects = data["objects"] if isinstance(data, dict) else data
        clouds = []
        for obj in objects:
            if "pcd_np" in obj:
                clouds.append(np.asarray(obj["pcd_np"]))
            elif "pcd" in obj:
                clouds.append(np.asarray(obj["pcd"].points))
        if not clouds:
            raise RuntimeError(
                "ConceptGraphs retained no objects after native multi-view filtering; increase recorded view coverage, not fabricated map geometry"
            )
        points = np.concatenate(clouds)
        record.update(
            {
                "frontend_command": front,
                "mapping_command": mapping,
                "native_map": str(native_map),
            }
        )
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
    ):
        raise RuntimeError("Invalid native map")
    np.save(output / "actual_map.npy", points.astype(np.float32))
    (output / "semantic-record.json").write_text(json.dumps(record, indent=2))
    print("ACTUAL NATIVE MAP:", method, len(points), "points", flush=True)


if __name__ == "__main__":
    main()
