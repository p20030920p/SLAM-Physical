"""Paired static C1/C2/C4 tests, with the input stream frozen at revision."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from slam_physical.geometry import (
    revised_poses,
    support_mask,
    surface_distance,
    world_points,
)
from slam_physical.native import run_beautymap
from slam_physical.storage import digest, publish_cloud, write_json

__all__ = ["run_experiment"]


def measure(
    points: np.ndarray,
    mask: np.ndarray,
    reference: np.ndarray,
    boxes: list[dict],
    labels: np.ndarray,
) -> dict:
    selected = points[mask]
    distances = surface_distance(selected, boxes)
    coverage = (
        (cKDTree(selected).query(reference, workers=2)[0] <= 0.05).mean()
        if len(selected)
        else 0.0
    )
    static = labels == 0
    dynamic = labels == 1
    sa = float(mask[static].mean() * 100) if static.any() else None
    da = float((~mask[dynamic]).mean() * 100) if dynamic.any() else None
    ha = (
        2 * sa * da / (sa + da)
        if sa is not None and da is not None and sa + da > 0
        else (0.0 if sa == 0 and da == 0 else None)
    )
    return {
        "retained_source_points": int(mask.sum()),
        "static_retention_percent": sa,
        "labelled_static_points": int(static.sum()),
        "labelled_dynamic_points": int(dynamic.sum()),
        "unclassified_points": int((labels < 0).sum()),
        "observed_surface_coverage_5cm_percent": float(coverage * 100),
        "primitive_surface_precision_5cm_percent": float(
            (distances <= 0.05).mean() * 100
        )
        if len(selected)
        else 0.0,
        "primitive_distance_p95_m": float(np.percentile(distances, 95))
        if len(selected)
        else None,
        "dynamic_accuracy": da,
        "HA": ha,
        "reason": "Dynamic denominator is zero"
        if not dynamic.any()
        else "SDF-labelled historical source points; ambiguous/unknown excluded",
    }


def run_experiment(
    dataset: Path, output: Path, upstream: Path, revision_after: int = 16
) -> dict:
    metadata = json.loads((dataset / "capture.json").read_text())
    if metadata.get("backend") != "gazebo-harmonic" or not metadata.get(
        "pose_telemetry_verified"
    ):
        raise ValueError("Require actual Gazebo capture with verified pose telemetry")
    if digest(dataset / "scans.npz") != metadata["scans_sha256"]:
        raise ValueError("Captured scan checksum mismatch")
    if not 16 <= revision_after <= metadata["frames"]:
        raise ValueError(
            "Revision prefix must contain 16..captured frame count observations"
        )
    archive = np.load(dataset / "scans.npz", allow_pickle=False)
    truth = archive["poses"][:revision_after]
    scans = [archive[f"scan_{i:04d}"] for i in range(revision_after)]
    labels = np.concatenate(
        [
            archive[f"labels_{i:04d}"]
            if f"labels_{i:04d}" in archive
            else np.zeros(len(scans[i]), dtype=np.int8)
            for i in range(revision_after)
        ]
    )
    if revision_after < 16 or len(truth) != revision_after:
        raise ValueError("Revision prefix must contain at least 16 actual observations")
    output.mkdir(parents=True, exist_ok=False)
    truth_points, frame_ids = world_points(scans, truth)
    static_points = truth_points[labels == 0]
    if not len(static_points):
        raise ValueError("No independently labelled static reference points")
    cells = np.floor(static_points / 0.05).astype(np.int64)
    reference = static_points[np.unique(cells, axis=0, return_index=True)[1]]
    publish_cloud(
        output,
        "raw",
        truth_points,
        "Corrected historical prefix received; no future observations",
    )
    parameters = {"dis_range": 8, "xy_resolution": 0.5, "h_res": 0.2}
    corrected_mask, corrected_record = run_beautymap(
        scans, truth, output / "native-c4", upstream, parameters
    )
    summary = {
        "scope": "Pose-revision mechanism pilot; not paper benchmark or SLAM accuracy",
        "dataset_sha256": digest(dataset / "scans.npz"),
        "capture": metadata,
        "revision_after": revision_after,
        "input_observation_ids": list(range(revision_after)),
        "future_observations_used": 0,
        "C1": {
            "voxel_m": 0.2,
            "distinct_frame_support": 3,
            "rule": "Old native keep OR at least 3 distinct old-pose frames in the same voxel; then reproject",
        },
        "parameters": parameters,
        "conditions": [],
        "C4_native_seconds": corrected_record["seconds"],
    }
    for amplitude in (0.0, 0.1, 0.3):
        label = f"drift-{round(amplitude * 100):02d}cm"
        old_poses = revised_poses(truth, amplitude)
        old_points, _ = world_points(scans, old_poses)
        publish_cloud(
            output, "raw", old_points, f"{label}: actual native old-pose classification"
        )
        old_mask, record = run_beautymap(
            scans, old_poses, output / f"native-{label}", upstream, parameters
        )
        protected = support_mask(old_points, frame_ids, 0.2, 3)
        groups = {
            "C0": (old_points, old_mask),
            "C1": (truth_points, old_mask | protected),
            "C2": (truth_points, old_mask),
            "C4": (truth_points, corrected_mask),
        }
        condition = {
            "amplitude_m": amplitude,
            "changed_observation_ids": list(
                np.flatnonzero(np.max(np.abs(old_poses - truth), axis=(1, 2)) > 0)
            ),
            "old_native_seconds": record["seconds"],
            "groups": {},
        }
        condition["changed_observation_ids"] = [
            int(i) for i in condition["changed_observation_ids"]
        ]
        for name, (points, mask) in groups.items():
            group_path = output / label / name
            group_path.mkdir(parents=True)
            np.savez_compressed(
                group_path / "map.npz",
                points=points[mask],
                source_ids=np.flatnonzero(mask),
            )
            metrics = measure(points, mask, reference, metadata["static_boxes"], labels)
            write_json(group_path / "metrics.json", metrics)
            condition["groups"][name] = metrics
            publish_cloud(
                output, name, points[mask], f"{label}: {name} finished, actual map"
            )
        condition["C4_recovers_C2_deleted_source_points"] = int(
            (corrected_mask & ~old_mask).sum()
        )
        condition["C4_deletes_C2_retained_source_points"] = int(
            (old_mask & ~corrected_mask).sum()
        )
        condition["C4_recovers_C2_deleted_static_source_points"] = int(
            (corrected_mask & ~old_mask & (labels == 0)).sum()
        )
        condition["C4_recovers_C2_deleted_dynamic_source_points"] = int(
            (corrected_mask & ~old_mask & (labels == 1)).sum()
        )
        if amplitude == 0 and not np.array_equal(old_mask, corrected_mask):
            raise RuntimeError("Zero-revision native equivalence gate failed")
        summary["conditions"].append(condition)
        write_json(output / "summary.json", summary)
    summary["zero_revision_equivalence"] = True
    summary["status"] = "completed_" + metadata.get("scenario", "static") + "_pilot"
    summary["interpretation"] = (
        "C4-C2 tests historical-decision sensitivity; C1 may match or outperform. No H1 success asserted."
    )
    write_json(output / "summary.json", summary)
    from slam_physical.report import render_report

    render_report(output)
    print(json.dumps(summary["conditions"], indent=2), flush=True)
    return summary
