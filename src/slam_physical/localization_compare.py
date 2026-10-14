"""Paired input replay: one completed test drive, fresh AMCL for each map."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from slam_physical.localization_session import session
from slam_physical.storage import digest, write_json

__all__ = ["compare", "main"]


def compare(build: Path, recording: Path, output: Path, gui=False, repeats=1):
    if repeats < 1:
        raise ValueError("At least one repeat is required")
    manifest = json.loads((build / "maps.json").read_text())
    if Path(manifest["mapping_recording"]).resolve() == recording.resolve():
        raise ValueError(
            "Use a separate held-out drive; this recording was used to build the maps"
        )
    output.mkdir(parents=True, exist_ok=False)
    source = json.loads((recording / "session.json").read_text())
    bag_hashes = {path.name: digest(path) for path in (recording / "bag").glob("*.db3")}
    rows = []
    jobs = [
        (repeat, method, record)
        for repeat in range(1, repeats + 1)
        for method, record in manifest["methods"].items()
    ]
    for repeat, method, record in jobs:
        if record["status"] != "completed":
            continue
        if (output / "STOP").exists():
            break
        run = output / (f"repeat-{repeat:02d}" if repeats > 1 else "") / method
        print(f"PAIRED REPLAY {repeat}/{repeats}: " + method, flush=True)
        try:
            session(run, Path(record["map_yaml"]), recording, gui)
            summary = json.loads((run / "localization_summary.json").read_text())
            rows.append(
                {
                    "method": method,
                    "repeat": repeat,
                    "family": record["family"],
                    "run": str(run),
                    **summary,
                }
            )
        except Exception as error:
            rows.append(
                {
                    "method": method,
                    "repeat": repeat,
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
                "requested_repeats": repeats,
                "mapping_scene": manifest.get("scenario", "unknown"),
                "mapping_pose_source": manifest["pose_source"],
                "test_scene": source.get("scenario", "static"),
                "test_bag_metadata_sha256": digest(recording / "bag/metadata.yaml"),
                "test_bag_files_sha256": bag_hashes,
                "amcl_config": "config/localization.yaml",
                "rows": rows,
                "scope": "same recorded sensor inputs; offline map geometry effects only; RGB-D semantic features are not consumed by AMCL",
            },
        )
    fields = [
        "method",
        "family",
        "repeat",
        "status",
        "matched_pose_updates",
        "position_rmse_m",
        "position_p95_m",
        "position_max_m",
        "post_warmup_5s_position_rmse_m",
        "post_warmup_5s_samples",
        "position_over_0_5m_fraction",
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
    expected = repeats * sum(
        item["status"] == "completed" for item in manifest["methods"].values()
    )
    if len(rows) != expected:
        raise InterruptedError(
            f"Comparison stopped after {len(rows)}/{expected} replays; saved partial data retained"
        )
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
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()
    rows = compare(
        args.maps.resolve(),
        args.recording.resolve(),
        args.output.resolve(),
        args.gui,
        args.repeats,
    )
    if not rows or any(row["status"] != "scored" for row in rows):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
