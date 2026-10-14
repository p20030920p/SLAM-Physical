"""Observed localization error and delivery timing; no trajectory alignment."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from slam_physical.storage import write_json

__all__ = ["localization_report"]


def localization_report(run: Path):
    with (run / "amcl_samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (run / "live_error.csv").open() as stream:
        live = list(csv.DictReader(stream))

    def numbers(key):
        return np.array([float(row[key]) for row in rows if row[key] not in ("", None)])

    errors = numbers("xy_error_m")
    yaw = numbers("yaw_error_rad")
    latency = numbers("scan_to_pose_observed_wall_ms")
    gaps = numbers("pose_gap_wall_s")
    stamps = numbers("stamp_sim_s")
    settled = errors[stamps >= stamps.min() + 5] if len(stamps) else errors
    summary = {
        "status": "scored" if len(errors) else "no_synchronized_localization_samples",
        "matched_pose_updates": len(rows),
        "live_prediction_samples": len(live),
        "position_rmse_m": float(np.sqrt((errors**2).mean())) if len(errors) else None,
        "position_p95_m": float(np.percentile(errors, 95)) if len(errors) else None,
        "position_max_m": float(errors.max()) if len(errors) else None,
        "post_warmup_5s_position_rmse_m": float(np.sqrt((settled**2).mean()))
        if len(settled)
        else None,
        "post_warmup_5s_samples": len(settled),
        "position_over_0_5m_fraction": float(np.mean(errors > 0.5))
        if len(errors)
        else None,
        "yaw_rmse_deg": float(np.sqrt((yaw**2).mean()) * 180 / np.pi)
        if len(yaw)
        else None,
        "observed_scan_to_pose_wall_p50_ms": float(np.percentile(latency, 50))
        if len(latency)
        else None,
        "observed_scan_to_pose_wall_p95_ms": float(np.percentile(latency, 95))
        if len(latency)
        else None,
        "pose_delivery_gap_wall_p95_s": float(np.percentile(gaps, 95))
        if len(gaps)
        else None,
        "timing_scope": "local observer scan-arrival to matching AMCL pose-arrival, including scheduling; not internal AMCL CPU runtime",
        "accuracy_scope": "same simulator map coordinates and timestamps, without SE2 alignment; evaluation truth is not supplied to AMCL",
        "stationary_note": "AMCL movement thresholds suppress updates when stationary; gaps alone are not failures",
    }
    if (run / "resources.csv").is_file():
        with (run / "resources.csv").open() as stream:
            resources = list(csv.DictReader(stream))
        if resources:
            summary.update(
                amcl_process_tree_cpu_mean_one_core_percent=float(
                    np.mean(
                        [
                            float(row["amcl_tree_cpu_one_core_percent"])
                            for row in resources
                        ]
                    )
                ),
                amcl_process_tree_rss_peak_mib=float(
                    max(float(row["amcl_tree_rss_mib"]) for row in resources)
                ),
                observed_real_time_factor_median=float(
                    np.median(
                        [float(row["observed_real_time_factor"]) for row in resources]
                    )
                ),
                cpu_scope="owned ros2 run wrapper and AMCL descendant processes; 100 percent means one CPU core",
            )
    write_json(run / "localization_summary.json", summary)
    if live:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        values = {
            key: np.array([float(row[key]) for row in live])
            for key in [
                "truth_x",
                "truth_y",
                "estimate_x",
                "estimate_y",
                "stamp_sim_s",
                "xy_error_m",
            ]
        }
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        axes[0].plot(
            values["truth_x"], values["truth_y"], color="green", label="Gazebo truth"
        )
        axes[0].plot(
            values["estimate_x"],
            values["estimate_y"],
            color="dodgerblue",
            label="AMCL + odometry",
        )
        axes[0].set_aspect("equal")
        axes[0].legend()
        axes[0].set_xlabel("Map X (m)")
        axes[0].set_ylabel("Map Y (m)")
        axes[1].plot(
            values["stamp_sim_s"] - values["stamp_sim_s"][0], values["xy_error_m"] * 100
        )
        axes[1].set_xlabel("Elapsed simulation time (s)")
        axes[1].set_ylabel("Live XY error (cm)")
        fig.tight_layout()
        fig.savefig(run / "localization.png", dpi=150)
        plt.close(fig)
    return summary
