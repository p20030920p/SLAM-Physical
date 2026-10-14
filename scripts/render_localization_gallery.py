"""Render README MP4/GIF clips from frozen localization traces, without ROS.

Usage (in the prepared LiDAR Python environment):
    python scripts/render_localization_gallery.py --benchmark runs/<benchmark> \
        --output ../SLAM_Learning/results/localization/static-dynamic
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Rectangle
from PIL import Image

METHODS = ("raw", "beautymap", "dufomap", "rgbd-raw", "conceptgraphs", "hovsg")
LABELS = ("LiDAR raw", "BeautyMap", "DUFOMap", "RGB-D raw", "ConceptGraphs", "HOV-SG")
COLORS = ("#515b65", "#b83360", "#007f8b")
ACTORS = {
    "west_cart": (0.65, 1.4),
    "south_cart": (1.4, 0.65),
    "north_cart": (1.5, 0.65),
}
PRESETS = (
    ("static", "static", METHODS, "Pure static scene | static maps + static test"),
    (
        "dynamic-lidar",
        "dynamic",
        METHODS[:3],
        "Dynamic scene | LiDAR maps + common dynamic test",
    ),
    (
        "dynamic-rgbd",
        "dynamic",
        METHODS[3:],
        "Dynamic scene | RGB-D geometry maps + common dynamic test",
    ),
    (
        "clean-map-dynamic-test",
        "clean-map-dynamic-test",
        METHODS,
        "Control | clean static maps + the SAME dynamic test",
    ),
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def encode_gif(video, output):
    # GIF uses 10 ms ticks: ffmpeg alternates 120/130 ms for 8 fps.
    command = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-nostdin",
        "-threads",
        "1",
        "-i",
        str(video),
        "-filter_complex_threads",
        "1",
        "-filter_complex",
        "[0:v]split[a][b];[a]palettegen=max_colors=256:stats_mode=diff[p];"
        "[b][p]paletteuse=dither=sierra2_4a:diff_mode=rectangle",
        "-threads",
        "1",
        "-vsync",
        "0",
        "-loop",
        "0",
        "-gifflags",
        "+transdiff",
        str(output),
    ]
    subprocess.run(command, check=True)
    with Image.open(output) as gif:
        delays = []
        for index in range(gif.n_frames):
            gif.seek(index)
            gif.load()
            delays.append(gif.info["duration"])
        metadata = {
            "dimensions": list(gif.size),
            "frames": gif.n_frames,
            "duration_seconds": sum(delays) / 1000,
            "delay_counts_ms": dict(Counter(delays)),
            "loop": gif.info.get("loop"),
            "full_decode_passed": True,
        }
    source = json.loads(
        subprocess.check_output(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=nb_frames,r_frame_rate:format=duration",
                "-of",
                "json",
                str(video),
            ]
        )
    )
    metadata["source_video_metadata"] = source
    if metadata["frames"] != int(source["streams"][0]["nb_frames"]):
        raise ValueError("GIF frame loss: " + str(output))
    if abs(metadata["duration_seconds"] - float(source["format"]["duration"])) > 0.011:
        raise ValueError("GIF duration changed: " + str(output))
    return metadata


def render(benchmark, output, preset, summary):
    name, condition, methods, title = preset
    comparison_path = benchmark / f"comparison-{condition}/comparison.json"
    comparison = json.loads(comparison_path.read_text())
    rows = {row["method"]: row for row in comparison["rows"] if row["repeat"] == 1}
    values, sources = {}, {}
    evidence = output / "evidence" / condition
    evidence.mkdir(parents=True, exist_ok=True)
    for method in methods:
        source = Path(rows[method]["run"]) / "live_error.csv"
        target = evidence / f"{method}.csv"
        shutil.copyfile(source, target)
        trace = read_csv(source)
        values[method] = {
            key: np.asarray([float(row[key]) for row in trace]) for key in trace[0]
        }
        sources[method] = {
            "trace": target.relative_to(output).as_posix(),
            "trace_sha256": digest(source),
        }
    start = max(value["stamp_sim_s"][0] for value in values.values())
    end = min(value["stamp_sim_s"][-1] for value in values.values())
    actors_path = benchmark / "recordings/dynamic-test/actor_observations.csv"
    actors = {}
    if condition != "static":
        actor_rows = read_csv(actors_path)
        for actor in ACTORS:
            selected = [row for row in actor_rows if row["name"] == actor]
            actors[actor] = {
                key: np.asarray([float(row[key]) for row in selected])
                for key in ("stamp_sim_s", "x", "y")
            }
        shutil.copyfile(actors_path, output / "evidence/actor_observations.csv")
    families = [methods[i : i + 3] for i in range(0, len(methods), 3)]
    figure = plt.figure(figsize=(12, 4.85 * len(families) + 0.75), layout="constrained")
    grid = figure.add_gridspec(
        2 * len(families), 3, height_ratios=[3.2, 1.1] * len(families)
    )
    artists, cursors = {}, []
    for group_index, group in enumerate(families):
        curve = figure.add_subplot(grid[2 * group_index + 1, :])
        curve.set(
            xlim=(0, end - start),
            ylim=(0, 32),
            xlabel="Recorded test time (s)",
            ylabel="Live XY error (cm)",
        )
        curve.axvspan(0, 5, color="#e5e7eb", alpha=0.8)
        curve.text(0.3, 26, "Initial convergence", fontsize=8, color="#5f6772")
        curve.grid(alpha=0.2)
        cursors.append(curve.axvline(0, color="#263442", alpha=0.6, linewidth=1))
        for column, method in enumerate(group):
            axis = figure.add_subplot(grid[2 * group_index, column])
            map_scene = "dynamic" if condition == "dynamic" else "static"
            map_dir = benchmark / f"maps-{map_scene}/maps" / method
            spec = yaml.safe_load((map_dir / "map.yaml").read_text())
            image = Image.open(map_dir / spec["image"])
            x, y = spec["origin"][:2]
            resolution = spec["resolution"]
            axis.imshow(
                image,
                cmap="gray",
                vmin=0,
                vmax=254,
                extent=[
                    x,
                    x + image.width * resolution,
                    y,
                    y + image.height * resolution,
                ],
            )
            axis.set(
                xlim=(0, 12),
                ylim=(0, 12),
                aspect="equal",
                xlabel="x (m)",
                ylabel="y (m)",
            )
            score = (
                summary[(condition, method)]["post_warmup_5s_position_rmse_m_mean"]
                * 100
            )
            axis.set_title(
                f"{LABELS[METHODS.index(method)]} | scored RMSE {score:.2f} cm",
                fontsize=10,
            )
            (truth,) = axis.plot([], [], color="#159447", linewidth=2)
            (estimate,) = axis.plot([], [], color="#2878d1", linewidth=1.5)
            (truth_dot,) = axis.plot([], [], "o", color="#159447", markersize=4)
            (estimate_dot,) = axis.plot([], [], "o", color="#2878d1", markersize=3)
            live_text = axis.text(
                0.025,
                0.97,
                "",
                transform=axis.transAxes,
                va="top",
                fontsize=9,
                bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"},
            )
            patches = {}
            for actor, size in ACTORS.items():
                if actor not in actors:
                    continue
                patch = Rectangle(
                    (0, 0), *size, color="#ed8723", alpha=0.55, visible=False
                )
                axis.add_patch(patch)
                patches[actor] = patch
            (line,) = curve.plot(
                [],
                [],
                color=COLORS[column],
                linewidth=1.5,
                label=LABELS[METHODS.index(method)],
            )
            artists[method] = (
                truth,
                estimate,
                truth_dot,
                estimate_dot,
                live_text,
                patches,
                line,
            )
            sources[method].update(
                {
                    "map_scene": map_scene,
                    "map_image_sha256": digest(map_dir / spec["image"]),
                    "map_yaml_sha256": digest(map_dir / "map.yaml"),
                }
            )
        curve.legend(loc="upper right", ncol=3, fontsize=8)
    figure.suptitle(
        title + "\nGreen: truth | blue: AMCL + odometry | orange: measured moving boxes"
        "\nRecorded-data reconstruction, repeat 1; scored RMSE uses AMCL updates after the first 5 s",
        fontsize=11,
    )
    video = output / f"{name}.mp4"
    writer = FFMpegWriter(
        fps=8,
        codec="libx264",
        bitrate=2000,
        extra_args=[
            "-pix_fmt",
            "yuv420p",
            "-vf",
            "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            "-movflags",
            "+faststart",
            "-threads",
            "2",
        ],
    )
    stamps = np.arange(start, end, 1 / 8)
    with writer.saving(figure, str(video), dpi=90):
        for stamp in stamps:
            for method, value in values.items():
                count = np.searchsorted(value["stamp_sim_s"], stamp, side="right")
                truth, estimate, truth_dot, estimate_dot, live_text, patches, line = (
                    artists[method]
                )
                truth.set_data(value["truth_x"][:count], value["truth_y"][:count])
                estimate.set_data(
                    value["estimate_x"][:count], value["estimate_y"][:count]
                )
                truth_dot.set_data(
                    value["truth_x"][count - 1 : count],
                    value["truth_y"][count - 1 : count],
                )
                estimate_dot.set_data(
                    value["estimate_x"][count - 1 : count],
                    value["estimate_y"][count - 1 : count],
                )
                live_text.set_text(
                    f"Live XY error: {value['xy_error_m'][count - 1] * 100:.1f} cm"
                )
                line.set_data(
                    value["stamp_sim_s"][:count] - start,
                    value["xy_error_m"][:count] * 100,
                )
                for actor, patch in patches.items():
                    observed = actors[actor]
                    index = (
                        np.searchsorted(observed["stamp_sim_s"], stamp, side="right")
                        - 1
                    )
                    if index >= 0:
                        size = ACTORS[actor]
                        patch.set_visible(True)
                        patch.set_xy(
                            (
                                observed["x"][index] - size[0] / 2,
                                observed["y"][index] - size[1] / 2,
                            )
                        )
            for cursor in cursors:
                cursor.set_xdata([stamp - start, stamp - start])
            writer.grab_frame()
    plt.close(figure)
    gif = output / f"{name}.gif"
    gif_info = encode_gif(video, gif)
    print(
        f"VERIFIED {name}: {gif_info['frames']} frames, {gif_info['duration_seconds']} s, {gif.stat().st_size} bytes",
        flush=True,
    )
    return {
        "name": name,
        "condition": condition,
        "methods": list(methods),
        "repeat": 1,
        "kind": "recorded-data reconstruction, not an RViz screen capture",
        "test_recording": Path(comparison["test_recording"]).name,
        "test_bag_files_sha256": comparison["test_bag_files_sha256"],
        "start_sim_s": float(start),
        "end_sim_s": float(end),
        "source_fps": 8,
        "sampling": "Continuous common trace interval at 8 fps; latest recorded sample; no cuts or speed-up",
        "curve_scope": "live AMCL+odometry prediction; table RMSE scores AMCL pose updates after initial 5 seconds",
        "sources": sources,
        "actor_observations_sha256": digest(actors_path) if actors else None,
        "mp4": video.name,
        "mp4_sha256": digest(video),
        "gif": gif.name,
        "gif_sha256": digest(gif),
        "gif_metadata": gif_info,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    benchmark, output = args.benchmark.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary_path = benchmark / "summary.json"
    summary = {
        (row["condition"], row["method"]): row
        for row in json.loads(summary_path.read_text())["rows"]
    }
    shutil.copyfile(benchmark / "summary.csv", output / "summary.csv")
    manifest = {
        "benchmark": benchmark.name,
        "renderer_sha256": digest(Path(__file__)),
        "summary_json_sha256": digest(summary_path),
        "summary_csv_sha256": digest(output / "summary.csv"),
        "protocol": "Independent mapping A and test B; provided poses only for mapping A; truth only for evaluation/display in B",
        "repeats": "Static/dynamic: 3 identical-input reproducibility checks; control: 1; GIFs show repeat 1",
        "clips": [],
    }
    for preset in PRESETS:
        manifest["clips"].append(render(benchmark, output, preset, summary))
        (output / "timing.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )


if __name__ == "__main__":
    main()
