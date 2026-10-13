"""Portable experiment files and atomic progress updates."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

__all__ = ["write_json", "digest", "write_pcd", "publish_cloud", "read_pcd"]


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    temporary.replace(path)


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_pcd(path: Path, xyz: np.ndarray, origin=(0.0, 0.0, 0.0)) -> None:
    points = np.asarray(xyz, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("PCD requires finite Nx3 coordinates")
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT "
        + " ".join(str(float(v)) for v in origin)
        + " 1 0 0 0\n"
        + f"POINTS {len(points)}\nDATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.tobytes())


def read_pcd(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        metadata = {}
        while True:
            line = stream.readline().decode("ascii").strip()
            if not line:
                raise ValueError("Incomplete PCD header")
            if line.startswith("#"):
                continue
            key, *values = line.split()
            metadata[key] = values
            if key == "DATA":
                break
        kinds = {("F", 4): "f4", ("F", 8): "f8", ("U", 4): "u4", ("I", 4): "i4"}
        dtype = np.dtype(
            [
                (name, "<" + kinds[(kind, int(size))])
                for name, kind, size in zip(
                    metadata["FIELDS"], metadata["TYPE"], metadata["SIZE"]
                )
            ]
        )
        count = int(metadata["POINTS"][0])
        if metadata["DATA"] == ["binary"]:
            array = np.frombuffer(stream.read(), dtype=dtype, count=count)
            return np.column_stack([array[name] for name in ("x", "y", "z")]).copy()
        if metadata["DATA"] == ["ascii"]:
            return np.loadtxt(stream, ndmin=2)[:, :3]
        raise ValueError(
            "Unsupported compressed PCD; decode with the native environment"
        )


def publish_cloud(run: Path, name: str, xyz: np.ndarray, stage: str) -> None:
    view = run / "view"
    view.mkdir(exist_ok=True)
    path = view / f"{name}.npy"
    temporary = view / f"{name}.tmp.npy"
    np.save(temporary, np.asarray(xyz, dtype=np.float32))
    temporary.replace(path)
    state_path = run / "viewer.json"
    state = (
        json.loads(state_path.read_text()) if state_path.exists() else {"clouds": {}}
    )
    state["clouds"][name] = {
        "path": str(path),
        "sha256": digest(path),
        "points": len(xyz),
    }
    state["stage"] = stage
    write_json(state_path, state)
