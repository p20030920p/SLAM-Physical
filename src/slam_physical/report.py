"""A measured result table and map figure; never infer success from appearance."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

__all__ = ["render_report"]


def render_report(output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    summary = json.loads((output / "summary.json").read_text())
    rows = []
    for condition in summary["conditions"]:
        for name, metrics in condition["groups"].items():
            rows.append(
                {
                    "drift_cm": int(round(condition["amplitude_m"] * 100)),
                    "group": name,
                    **{k: v for k, v in metrics.items() if k != "reason"},
                }
            )
    with (output / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    fig, axes = plt.subplots(1, 4, figsize=(16, 4), sharex=True, sharey=True)
    for ax, name in zip(axes, ["C0", "C1", "C2", "C4"]):
        points = np.load(output / "drift-30cm" / name / "map.npz", allow_pickle=False)[
            "points"
        ]
        points = points[:: max(1, len(points) // 20000)]
        ax.scatter(points[:, 0], points[:, 1], s=0.2)
        ax.set_title(name)
        ax.set_xlabel("World X (m)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    axes[0].set_ylabel("World Y (m)")
    scenario = summary["capture"].get("scenario", "static")
    fig.suptitle(
        f"Actual Gazebo / native BeautyMap | 30 cm historical drift | {scenario} pilot"
    )
    fig.tight_layout()
    fig.savefig(output / "maps.png", dpi=150)
    plt.close(fig)
    lines = [
        f"# Measured {scenario} pilot",
        "",
        "Not a paper benchmark or an H1 success claim.",
        "",
        "| Drift cm | Group | SA % | DA % | HA % | Observed coverage % | Surface precision % |",
        "|---:|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        da = (
            f"{row['dynamic_accuracy']:.4f}"
            if row["dynamic_accuracy"] is not None
            else "undefined"
        )
        ha = f"{row['HA']:.4f}" if row["HA"] is not None else "undefined"
        lines.append(
            f"| {row['drift_cm']} | {row['group']} | {row['static_retention_percent']:.4f} | {da} | {ha} | {row['observed_surface_coverage_5cm_percent']:.4f} | {row['primitive_surface_precision_5cm_percent']:.4f} |"
        )
    lines += [
        "",
        "C1 uses fixed old-coordinate multi-frame protection. C2 freezes native keep/delete decisions.",
        "C4 is a full native rerun on the corrected historical prefix. Future observations used: 0.",
        "Coverage reference uses the same sensor samples with simulator poses; primitive distances use independent SDF box geometry.",
        "SDF labels are evaluation only; native input has XYZ and sensor viewpoint only. Ambiguous/unknown labels are excluded from SA/DA.",
        "When the dynamic denominator is zero, DA and HA are undefined.",
        "",
        "![Actual maps](maps.png)",
    ]
    (output / "REPORT.md").write_text("\n".join(lines) + "\n")
