"""Paired input replay: one completed test drive, fresh AMCL for each map."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from slam_physical.localization_session import session
from slam_physical.storage import write_json

__all__ = ["compare", "main"]


def compare(build: Path, recording: Path, output: Path, gui=False):
    manifest = json.loads((build / "maps.json").read_text())
    if Path(manifest["mapping_recording"]).resolve() == recording.resolve():
        raise ValueError(
            "Use a separate held-out drive; this recording was used to build the maps"
        )
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for method, record in manifest["methods"].items():
        if record["status"] != "completed":
            continue
        if (output / "STOP").exists():
            break
        run = output / method
        print("PAIRED REPLAY: " + method, flush=True)
        try:
            session(run, Path(record["map_yaml"]), recording, gui)
            summary = json.loads((run / "localization_summary.json").read_text())
            rows.append({"method": method, "family": record["family"], **summary})
        except Exception as error:
            rows.append(
                {
                    "method": method,
                    "family": record["family"],
                    "status": "failed",
                    "error": str(error),
                }
            )
        write_json(
            output / "comparison.json",
            {
                "mapping_recording": manifest["mapping_recording"],
                "test_recording": str(recording),
                "input_replay_rate": 1.0,
                "amcl_config": "config/localization.yaml",
                "rows": rows,
                "scope": "same recorded sensor inputs; offline map geometry effects only; RGB-D semantic features are not consumed by AMCL",
            },
        )
    fields = [
        "method",
        "family",
        "status",
        "matched_pose_updates",
        "position_rmse_m",
        "position_p95_m",
        "yaw_rmse_deg",
        "observed_scan_to_pose_wall_p50_ms",
        "observed_scan_to_pose_wall_p95_ms",
        "pose_delivery_gap_wall_p95_s",
        "amcl_process_tree_cpu_mean_one_core_percent",
        "amcl_process_tree_rss_peak_mib",
        "observed_real_time_factor_median",
    ]
    with (output / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    if not rows or any(row["status"] != "scored" for row in rows):
        print(
            "Some replays failed or were stopped; inspect comparison.json. No missing result is replaced with a score.",
            flush=True,
        )
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--maps", type=Path, required=True)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    compare(
        args.maps.resolve(), args.recording.resolve(), args.output.resolve(), args.gui
    )


if __name__ == "__main__":
    main()
