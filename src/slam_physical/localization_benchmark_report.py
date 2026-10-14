"""Evidence and repeated-input comparisons for static/dynamic localization."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation

from slam_physical.dynamic_scene import ACTORS
from slam_physical.occupancy import GRID, reference_grid
from slam_physical.storage import digest, write_json

METHODS = ["raw", "beautymap", "dufomap", "rgbd-raw", "conceptgraphs", "hovsg"]
CONDITIONS = ["static", "dynamic", "clean-map-dynamic-test"]


def load_csv(path: Path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def aggregate(rows: list[dict], condition: str) -> list[dict]:
    result = []
    for method in METHODS:
        selected = [row for row in rows if row["method"] == method]
        if not selected or any(row["status"] != "scored" for row in selected):
            raise ValueError(
                f"Missing/failed localization result: {condition}/{method}"
            )
        record = {
            "condition": condition,
            "method": method,
            "family": selected[0]["family"],
            "repeats": len(selected),
        }
        for key in [
            "position_rmse_m",
            "post_warmup_5s_position_rmse_m",
            "position_p95_m",
            "position_max_m",
            "yaw_rmse_deg",
            "observed_scan_to_pose_wall_p95_ms",
        ]:
            values = np.asarray([row[key] for row in selected], dtype=float)
            if not np.isfinite(values).all():
                raise ValueError("Non-finite metric: " + key)
            record[key + "_mean"] = float(values.mean())
            record[key + "_std"] = (
                float(values.std(ddof=1)) if len(values) > 1 else None
            )
        result.append(record)
    for record in result:
        baseline = next(
            item
            for item in result
            if item["method"] == ("raw" if record["family"] == "lidar" else "rgbd-raw")
        )
        raw = baseline["post_warmup_5s_position_rmse_m_mean"]
        value = record["post_warmup_5s_position_rmse_m_mean"]
        record["baseline"] = baseline["method"]
        record["post_warmup_rmse_reduction_m"] = raw - value
        record["post_warmup_rmse_reduction_percent"] = (
            (raw - value) / raw * 100 if raw else None
        )
    return result


def scene_evidence(recording: Path) -> dict:
    metadata = json.loads((recording / "session.json").read_text())
    actors = load_csv(recording / "actor_observations.csv")
    evidence = {
        "scenario": metadata["scenario"],
        "actor_observed_ranges": {},
        "bag_files_sha256": {
            p.name: digest(p) for p in (recording / "bag").glob("*.db3")
        },
    }
    for actor in ACTORS:
        values = np.array(
            [
                [float(row[k]) for k in ["stamp_sim_s", "x", "y", "z"]]
                for row in actors
                if row["name"] == actor["name"]
            ]
        )
        if len(values):
            evidence["actor_observed_ranges"][actor["name"]] = {
                "samples": len(values),
                "x_range_m": float(np.ptp(values[:, 1])),
                "y_range_m": float(np.ptp(values[:, 2])),
            }
    if metadata["scenario"] == "dynamic" and (
        len(evidence["actor_observed_ranges"]) != len(ACTORS)
        or any(
            max(item["x_range_m"], item["y_range_m"]) < 0.5
            for item in evidence["actor_observed_ranges"].values()
        )
    ):
        raise ValueError("Dynamic scene lacks independently observed moving obstacles")
    archive = np.load(recording / "mapping_scans.npz", allow_pickle=False)
    provided_path = recording / "provided_lidar_poses.npy"
    poses = (
        np.load(provided_path, allow_pickle=False)
        if provided_path.exists()
        else archive["poses"]
    )
    evidence["point_evaluation_pose_source"] = (
        "provided simulator poses"
        if provided_path.exists()
        else "wheel odometry (diagnostic only)"
    )
    total, moving = 0, 0
    for index, (stamp, pose) in enumerate(zip(archive["stamps"], poses)):
        if not np.isfinite(pose).all():
            continue
        points = archive[f"scan_{index:04d}"] @ pose[:3, :3].T + pose[:3, 3]
        selected = points[(points[:, 2] >= 0.5) & (points[:, 2] <= 0.8)]
        mask = np.zeros(len(selected), dtype=bool)
        for actor in ACTORS:
            matches = [
                row
                for row in actors
                if row["name"] == actor["name"]
                and abs(float(row["stamp_sim_s"]) - stamp) < 0.15
            ]
            if not matches:
                continue
            observed = min(
                matches, key=lambda row: abs(float(row["stamp_sim_s"]) - stamp)
            )
            center = np.array([float(observed[k]) for k in ["x", "y", "z"]])
            half = np.array(actor["size"]) / 2
            # Evaluation only. Account for discrete actor updates and odometry's
            # small geometric offset, never feed these labels to a mapper.
            delta = np.abs(selected - center) - half
            mask |= (delta.max(axis=1) < 0.12) & (np.abs(delta).min(axis=1) < 0.12)
        total += len(selected)
        moving += int(mask.sum())
    evidence.update(
        lidar_keyframes=len(archive["stamps"]),
        height_band_points=total,
        points_near_observed_actor_surfaces=moving,
        dynamic_surface_fraction=moving / total if total else None,
    )
    if metadata["scenario"] == "dynamic" and moving == 0:
        raise ValueError("Moving boxes were not observed in actual LiDAR inputs")
    return evidence


def report(benchmark: Path):
    aggregated, comparisons = [], {}
    for condition in CONDITIONS:
        value = json.loads(
            (benchmark / ("comparison-" + condition) / "comparison.json").read_text()
        )
        if len(value["rows"]) != len(METHODS) * value["requested_repeats"] or any(
            sum(row["method"] == method for row in value["rows"])
            != value["requested_repeats"]
            for method in METHODS
        ):
            raise ValueError("Incomplete repeated comparison: " + condition)
        comparisons[condition] = value
        aggregated.extend(aggregate(value["rows"], condition))
    evidence = {
        name: scene_evidence(benchmark / "recordings" / name)
        for name in ["static-mapping", "static-test", "dynamic-mapping", "dynamic-test"]
    }
    # Geometry labels are used only here, after native maps and AMCL runs exist.
    boxes = json.loads(
        (benchmark / "recordings/static-mapping/world.json").read_text()
    )["static_boxes"]
    static_occupied = reference_grid(boxes) == 0
    tolerance = binary_dilation(static_occupied, iterations=2)
    yy, xx = np.mgrid[: GRID["height"], : GRID["width"]]
    x, y = -1 + (xx + 0.5) * 0.05, -1 + (yy + 0.5) * 0.05
    interior = (x > 0.2) & (x < 11.8) & (y > 0.2) & (y < 11.8)
    map_metrics = []
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 6, figsize=(19, 7), constrained_layout=True)
    for row, scene in enumerate(["static", "dynamic"]):
        for column, method in enumerate(METHODS):
            path = benchmark / ("maps-" + scene) / "maps" / method / "map.pgm"
            image = np.asarray(Image.open(path))
            grid = np.flipud(image)
            map_metrics.append(
                {
                    "scene": scene,
                    "method": method,
                    "map_pgm_sha256": digest(path),
                    "occupied_cells_off_static_structure": int(
                        ((grid == 0) & ~tolerance & interior).sum()
                    ),
                    "scope": "evaluation only; cells >2 grid dilations from SDF static structure; includes projection/odometry errors, not exclusively dynamic ghosts",
                }
            )
            axes[row, column].imshow(
                image, cmap="gray", vmin=0, vmax=254, extent=[-1, 13, -1, 13]
            )
            axes[row, column].set_title(scene + " / " + method, fontsize=10)
            axes[row, column].set_xlim(0, 12)
            axes[row, column].set_ylim(0, 12)
            axes[row, column].set_aspect("equal")
    figure.savefig(benchmark / "map-gallery.png", dpi=135)
    plt.close(figure)
    figure, axes = plt.subplots(2, 3, figsize=(12, 8), constrained_layout=True)
    for row, scene in enumerate(["static", "dynamic"]):
        for column, method in enumerate(METHODS[:3]):
            axis = axes[row, column]
            axis.imshow(
                Image.open(benchmark / ("maps-" + scene) / "maps" / method / "map.pgm"),
                cmap="gray",
                vmin=0,
                vmax=254,
                extent=[-1, 13, -1, 13],
            )
            axis.set_title(scene + " / " + method)
            axis.set_xlim(0, 12)
            axis.set_ylim(0, 12)
            axis.set_aspect("equal")
    figure.savefig(benchmark / "lidar-map-comparison.png", dpi=140)
    plt.close(figure)
    figure, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    for column, condition in enumerate(CONDITIONS):
        for family_index, family in enumerate(["lidar", "rgbd_geometry_projection"]):
            axis = axes[family_index, column]
            rows = [
                row
                for row in aggregated
                if row["condition"] == condition and row["family"] == family
            ]
            axis.bar(
                np.arange(3),
                [row["post_warmup_5s_position_rmse_m_mean"] * 100 for row in rows],
                yerr=[
                    (row["post_warmup_5s_position_rmse_m_std"] or 0) * 100
                    for row in rows
                ],
                color=["#7c8798", "#2878b5", "#d95f02"],
                capsize=3,
            )
            axis.set_xticks(
                np.arange(3), [row["method"] for row in rows], rotation=20, ha="right"
            )
            axis.set_ylabel("Position RMSE after first 5 s (cm)")
            axis.set_title(
                condition + (" / LiDAR" if family_index == 0 else " / RGB-D geometry"),
                fontsize=10,
            )
            axis.grid(axis="y", alpha=0.2)
    figure.savefig(benchmark / "localization-comparison.png", dpi=160)
    plt.close(figure)
    write_json(
        benchmark / "summary.json",
        {
            "rows": aggregated,
            "recording_evidence": evidence,
            "map_geometry_evaluation": map_metrics,
            "scope": "Fixed recorded test path per scene; repetitions check reproducibility and any observed variation, not independent scene/route generalization. Identical outputs and zero spread do not imply zero generalization uncertainty. LiDAR and RGB-D compared within their own input families. Offline maps; no semantic localization or online neural inference.",
            "mapping_pose_source": comparisons["dynamic"]["mapping_pose_source"],
            "map_builds": {
                scene: json.loads(
                    (benchmark / ("maps-" + scene) / "maps.json").read_text()
                )
                for scene in ["static", "dynamic"]
            },
        },
    )
    with (benchmark / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(aggregated[0]))
        writer.writeheader()
        writer.writerows(aggregated)
    training_note = (
        "主实验建图 A 明确使用仿真提供的传感器位姿，隔离轮速漂移和动态过滤影响；它不是建图位姿估计算法或完整 SLAM 的评测。测试 B 的 AMCL 仅使用激光、轮速和待测地图，真值只用于评分/显示。较早的纯轮速建图结果保存在诊断目录，没有与给定位姿版本混在同一统计中。"
        if comparisons["dynamic"]["mapping_pose_source"].startswith(
            "simulator-provided"
        )
        else "本次使用轮速建图位姿，地图误差同时包含轮速漂移影响；测试 B 的 AMCL 仅使用激光、轮速和待测地图，真值只用于评分/显示。"
    )
    lines = [
        "# 静态与动态场景定位实测",
        "",
        "使用真实 Gazebo 传感器录制，地图来自作者算法的实际执行。每种场景分别采集建图 A 与测试 B；各方法回放相同 B。机器人由轮关节运动，障碍物由目标 10 Hz 的脚本轨迹驱动。",
        "",
        training_note,
        "",
        "主指标为去掉前 5 秒初始化后的位置 RMSE。重复运行检查同一输入下的可复现性和实际波动，不代表独立场景/路线样本。若输出一致，标准差照实为 0，这不表示泛化不确定性为 0。正的‘减少’表示相对本家族原始地图误差下降，负值表示上升。",
        "",
        "| 条件 | 地图 | 次数 | 初始化后 RMSE 均值 ± 标准差（cm） | 相对本家族基线减少（cm） |",
        "|---|---|---:|---:|---:|",
    ]
    for row in aggregated:
        mean = row["post_warmup_5s_position_rmse_m_mean"] * 100
        std = row["post_warmup_5s_position_rmse_m_std"]
        spread = f" ± {std * 100:.2f}" if std is not None else "（单次）"
        lines.append(
            f"| {row['condition']} | {row['method']} | {row['repeats']} | {mean:.2f}{spread} | {row['post_warmup_rmse_reduction_m'] * 100:+.2f} |"
        )
    lines += [
        "",
        "static：静态建图 + 静态测试。dynamic：动态建图 + 动态测试。clean-map-dynamic-test：静态干净地图 + 与 dynamic 完全相同的动态测试 B，用于分辨地图残影与实时动态遮挡。",
        "",
        "LiDAR 家族基线为 raw；RGB-D 家族基线为 rgbd-raw。CG/HOV-SG 只提供几何投影，AMCL 没有使用 CLIP 语义或层级关系。在线推理耗时不包含在回放中。",
        "",
        "![误差比较](localization-comparison.png)",
        "",
        "![实际占据地图](map-gallery.png)",
        "",
        "完整数值见 summary.csv、summary.json 和 comparison-*/comparison.csv。原始 bag、RGB-D、扫描、输入哈希及节点日志在 recordings/ 与 maps-*/ 中。",
    ]
    (benchmark / "REPORT.zh-CN.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print("BENCHMARK REPORT: " + str(benchmark / "REPORT.zh-CN.md"), flush=True)


def video(benchmark: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    from matplotlib.patches import Rectangle

    comparison = json.loads(
        (benchmark / "comparison-dynamic/comparison.json").read_text()
    )
    runs = {
        row["method"]: Path(row["run"])
        for row in comparison["rows"]
        if row["repeat"] == 1
    }
    values = {method: load_csv(run / "live_error.csv") for method, run in runs.items()}
    start = max(float(rows[0]["stamp_sim_s"]) for rows in values.values())
    end = min(float(rows[-1]["stamp_sim_s"]) for rows in values.values())
    actors = load_csv(benchmark / "recordings/dynamic-test/actor_observations.csv")
    figure, axes = plt.subplots(2, 3, figsize=(15, 9), constrained_layout=True)
    artists = {}
    for axis, method in zip(axes.flat, METHODS):
        image = Image.open(benchmark / "maps-dynamic/maps" / method / "map.pgm")
        axis.imshow(image, cmap="gray", vmin=0, vmax=254, extent=[-1, 13, -1, 13])
        axis.set_xlim(0, 12)
        axis.set_ylim(0, 12)
        axis.set_aspect("equal")
        (truth,) = axis.plot([], [], color="#1b9e39", linewidth=2, label="Truth")
        (estimate,) = axis.plot(
            [], [], color="#2878d1", linewidth=1.5, label="AMCL + odometry"
        )
        patches = []
        for actor in ACTORS:
            patch = Rectangle((0, 0), *actor["size"][:2], color="#f08020", alpha=0.5)
            axis.add_patch(patch)
            patches.append(patch)
        axis.legend(loc="lower left", fontsize=7)
        artists[method] = (axis, truth, estimate, patches)
    figure.suptitle(
        "Measured dynamic test replay | green: truth, blue: estimate, orange: observed obstacles\nReconstructed from recorded data; offline maps + common AMCL; first repeat",
        fontsize=11,
    )
    output = benchmark / "dynamic-localization-replay.mp4"
    writer = FFMpegWriter(
        fps=8,
        codec="libx264",
        bitrate=2000,
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )
    with writer.saving(figure, str(output), dpi=90):
        for stamp in np.arange(start, end, 1 / 8):
            for method, rows in values.items():
                selected = [row for row in rows if float(row["stamp_sim_s"]) <= stamp]
                axis, truth, estimate, patches = artists[method]
                truth.set_data(
                    [float(row["truth_x"]) for row in selected],
                    [float(row["truth_y"]) for row in selected],
                )
                estimate.set_data(
                    [float(row["estimate_x"]) for row in selected],
                    [float(row["estimate_y"]) for row in selected],
                )
                axis.set_title(
                    f"{method} | t={stamp-start:.1f}s | XY error={float(selected[-1]['xy_error_m'])*100:.1f} cm",
                    fontsize=10,
                )
                for actor, patch in zip(ACTORS, patches):
                    observed = [
                        row
                        for row in actors
                        if row["name"] == actor["name"]
                        and float(row["stamp_sim_s"]) <= stamp
                    ]
                    if observed:
                        patch.set_xy(
                            (
                                float(observed[-1]["x"]) - actor["size"][0] / 2,
                                float(observed[-1]["y"]) - actor["size"][1] / 2,
                            )
                        )
            writer.grab_frame()
    plt.close(figure)
    write_json(
        benchmark / "video.json",
        {
            "file": output.name,
            "kind": "recorded-data reconstruction, not an RViz screen capture",
            "test_recording": comparison["test_recording"],
            "repeat": 1,
            "fps": 8,
            "start_sim_s": start,
            "end_sim_s": end,
        },
    )
    print("RECORDED DATA VIDEO: " + str(output), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    (video if args.video else report)(args.benchmark.resolve())


if __name__ == "__main__":
    main()
