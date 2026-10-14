"""One declared 2D map projection for all geometric mapping methods."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation

from slam_physical.storage import write_json

__all__ = ["reference_grid", "project_cloud", "save_grid", "GRID"]
GRID = {
    "resolution": 0.05,
    "origin": [-1.0, -1.0, 0.0],
    "width": 280,
    "height": 280,
    "height_band": [0.50, 0.80],
    "dilation_cells": 1,
}


def save_grid(path: Path, grid: np.ndarray, provenance: dict) -> Path:
    path.mkdir(parents=True, exist_ok=False)
    image = np.flipud(grid).astype(np.uint8)
    with (path / "map.pgm").open("wb") as stream:
        stream.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode())
        stream.write(image.tobytes())
    (path / "map.yaml").write_text(
        "image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.0, -1.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    write_json(path / "projection.json", {"grid": GRID, **provenance})
    return path / "map.yaml"


def reference_grid(boxes: list[dict]) -> np.ndarray:
    yy, xx = np.mgrid[: GRID["height"], : GRID["width"]]
    x = GRID["origin"][0] + (xx + 0.5) * GRID["resolution"]
    y = GRID["origin"][1] + (yy + 0.5) * GRID["resolution"]
    grid = np.full(x.shape, 205, dtype=np.uint8)
    grid[(x >= 0) & (x <= 12) & (y >= 0) & (y <= 12)] = 254
    for box in boxes:
        center, half = np.array(box["center"]), np.array(box["size"]) / 2
        if center[2] - half[2] <= 0.65 <= center[2] + half[2]:
            occupied = (np.abs(x - center[0]) <= half[0]) & (
                np.abs(y - center[1]) <= half[1]
            )
            grid[occupied] = 0
    return grid


def project_cloud(
    points: np.ndarray, evidence_points: np.ndarray, origins: np.ndarray
) -> np.ndarray:
    """Free space from common raw rays; occupied cells from each method's output."""
    grid = np.full((GRID["height"], GRID["width"]), 205, dtype=np.uint8)
    low, high = GRID["height_band"]
    raw_mask = (evidence_points[:, 2] >= low) & (evidence_points[:, 2] <= high)
    for endpoint, origin in zip(evidence_points[raw_mask, :2], origins[raw_mask, :2]):
        count = max(2, int(np.linalg.norm(endpoint - origin) / GRID["resolution"]) + 1)
        ray = origin + np.linspace(0, 1, count)[:, None] * (endpoint - origin)
        cells = np.floor((ray - GRID["origin"][:2]) / GRID["resolution"]).astype(int)
        valid = (
            (cells[:, 0] >= 0)
            & (cells[:, 0] < GRID["width"])
            & (cells[:, 1] >= 0)
            & (cells[:, 1] < GRID["height"])
        )
        cells = cells[valid]
        grid[cells[:, 1], cells[:, 0]] = 254
    selected = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
    cells = np.floor(
        (selected[:, :2] - GRID["origin"][:2]) / GRID["resolution"]
    ).astype(int)
    valid = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < GRID["width"])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < GRID["height"])
    )
    cells = cells[valid]
    occupied = np.zeros(grid.shape, dtype=bool)
    occupied[cells[:, 1], cells[:, 0]] = True
    occupied = binary_dilation(occupied, iterations=GRID["dilation_cells"])
    grid[occupied] = 0
    return grid
