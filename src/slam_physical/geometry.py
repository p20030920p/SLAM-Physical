"""Pose transforms and independent primitive-surface distances."""

from __future__ import annotations

import numpy as np

__all__ = ["world_points", "revised_poses", "support_mask", "surface_distance"]


def world_points(
    scans: list[np.ndarray], poses: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    if len(scans) != len(poses):
        raise ValueError("One pose is required per scan")
    points = [p @ t[:3, :3].T + t[:3, 3] for p, t in zip(scans, poses)]
    return np.concatenate(points).astype(np.float32), np.concatenate(
        [np.full(len(p), i, dtype=np.int32) for i, p in enumerate(scans)]
    )


def revised_poses(truth: np.ndarray, amplitude: float) -> np.ndarray:
    old = truth.copy()
    for i in range(8, min(16, len(old))):
        old[i, 0, 3] += amplitude * (i - 8) / 7
    return old


def support_mask(
    points: np.ndarray, frame_ids: np.ndarray, voxel: float, minimum: int
) -> np.ndarray:
    """Distinct observing frames per old-coordinate voxel; no GT or labels."""
    cells = np.floor(points / voxel).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    frame_count = int(frame_ids.max()) + 1
    unique_pairs = np.unique(inverse.astype(np.int64) * frame_count + frame_ids)
    support = np.bincount(unique_pairs // frame_count, minlength=int(inverse.max()) + 1)
    return support[inverse] >= minimum


def surface_distance(points: np.ndarray, boxes: list[dict]) -> np.ndarray:
    """Exact Euclidean distance to fixed Gazebo box surfaces (not their volume)."""
    distance = np.full(len(points), np.inf)
    for box in boxes:
        q = np.abs(points - np.asarray(box["center"])) - np.asarray(box["size"]) / 2
        outside = np.linalg.norm(np.maximum(q, 0), axis=1)
        inside = -np.max(q, axis=1)
        distance = np.minimum(
            distance, np.where(np.max(q, axis=1) <= 0, inside, outside)
        )
    return distance
