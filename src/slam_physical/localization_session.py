"""Own one live/replayed AMCL session and its recorder, never another ROS session."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import yaml

from slam_physical.drive_world import build_drive_world
from slam_physical.dynamic_scene import ACTORS, drive_command
from slam_physical.occupancy import reference_grid, save_grid
from slam_physical.storage import digest, write_json

__all__ = ["main", "session"]
ROOT = Path(__file__).resolve().parents[2]


def session(
    run: Path,
    map_file: Path | None = None,
    replay: Path | None = None,
    gui=True,
    duration=0.0,
    autodrive=False,
    linear_scale=1.0,
    angular_scale=1.0,
    scenario="static",
    actor_phase=0.0,
    route="legacy",
):
    import rclpy
    from geometry_msgs.msg import Twist

    from slam_physical.drive_telemetry import DriveTelemetry
    from slam_physical.localization_report import localization_report
    from slam_physical.process_metrics import process_tree_counters

    run.mkdir(parents=True, exist_ok=False)
    domain = 120 + int(hashlib.sha256(str(run).encode()).hexdigest()[:4], 16) % 60
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["GZ_PARTITION"] = "slam_drive_" + uuid.uuid4().hex[:10]
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(
        ROOT / ".cache/sim2real/src/race_gazebo/models"
    )
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    os.environ.setdefault("MESA_GL_VERSION_OVERRIDE", "4.5")
    if replay:
        replay = replay.resolve()
        if not (replay / "bag/metadata.yaml").is_file():
            raise ValueError(
                "Replay needs a completed session containing bag/metadata.yaml"
            )
        source = json.loads((replay / "session.json").read_text())
        linear_scale, angular_scale = (
            source["odom_linear_scale"],
            source["odom_angular_scale"],
        )
        scenario = source.get("scenario", "static")
        actor_phase = source.get("actor_phase_s", 0.0)
    boxes = build_drive_world(
        run / "world.sdf", ROOT / ".cache/sim2real", scenario, actor_phase
    )
    if map_file is None:
        map_file = save_grid(
            run / "reference_map",
            reference_grid(boxes),
            {"type": "SDF reference baseline, not an algorithm map"},
        )
    map_file = map_file.resolve()
    if not map_file.is_file():
        raise FileNotFoundError(map_file)
    config = ROOT / "config/localization.yaml"
    write_json(
        run / "session.json",
        {
            "status": "starting",
            "mode": "replay" if replay else "live",
            "ros_domain_id": domain,
            "map_yaml": str(map_file),
            "map_yaml_sha256": digest(map_file),
            "map_image_sha256": digest(
                map_file.parent / yaml.safe_load(map_file.read_text())["image"]
            ),
            "amcl_config_sha256": digest(config),
            "source_recording": str(replay) if replay else None,
            "odom_linear_scale": linear_scale,
            "odom_angular_scale": angular_scale,
            "robot": "physics differential drive in Sim2Real measurement annex; no robot set_pose",
            "truth_role": "display/scoring only; reference baseline map is explicitly SDF-derived",
            "initial_pose": "fixed AMCL guess (6.25,6.15,0.12rad); robot spawn (6,6,0rad)",
            "scenario": scenario,
            "actor_phase_s": actor_phase,
            "route": route
            if autodrive
            else "keyboard"
            if not replay
            else source.get("route", "unknown"),
            "duration_sim_s_after_ready": duration,
            "dynamic_role": "scripted obstacle motion only; actor observations display/scoring, never AMCL input",
        },
    )
    children = []
    handles = []
    resource_file = (run / "resources.csv").open("w", newline="")
    resource_csv = csv.writer(resource_file)
    resource_csv.writerow(
        [
            "elapsed_wall_s",
            "simulation_time_s",
            "observed_real_time_factor",
            "amcl_tree_cpu_one_core_percent",
            "amcl_tree_rss_mib",
        ]
    )
    rclpy.init()
    node = DriveTelemetry(
        run, linear_scale, angular_scale, collect=not bool(replay), scenario=scenario
    )
    automation = node.create_publisher(Twist, "/teleop/cmd_vel", 10)

    def start(name, command):
        print("START " + name + ": " + " ".join(map(str, command)), flush=True)
        log = (run / (name + ".log")).open("w")
        handles.append(log)
        process = subprocess.Popen(
            list(map(str, command)),
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        children.append((name, process))
        return process

    try:
        if not replay:
            start(
                "gazebo",
                ["gz", "sim", "-r", "-s", "--headless-rendering", run / "world.sdf"],
            )
            bridges = [
                {
                    "ros_topic_name": "/scan",
                    "gz_topic_name": "/scan",
                    "ros_type_name": "sensor_msgs/msg/LaserScan",
                    "gz_type_name": "gz.msgs.LaserScan",
                    "direction": "GZ_TO_ROS",
                },
                {
                    "ros_topic_name": "/lidar3d/points",
                    "gz_topic_name": "/lidar3d/points",
                    "ros_type_name": "sensor_msgs/msg/PointCloud2",
                    "gz_type_name": "gz.msgs.PointCloudPacked",
                    "direction": "GZ_TO_ROS",
                },
                {
                    "ros_topic_name": "/clock",
                    "gz_topic_name": "/clock",
                    "ros_type_name": "rosgraph_msgs/msg/Clock",
                    "gz_type_name": "gz.msgs.Clock",
                    "direction": "GZ_TO_ROS",
                },
                {
                    "ros_topic_name": "/wheel_odom",
                    "gz_topic_name": "/wheel_odom",
                    "ros_type_name": "nav_msgs/msg/Odometry",
                    "gz_type_name": "gz.msgs.Odometry",
                    "direction": "GZ_TO_ROS",
                },
                {
                    "ros_topic_name": "/truth",
                    "gz_topic_name": "/model/drive_robot/pose",
                    "ros_type_name": "tf2_msgs/msg/TFMessage",
                    "gz_type_name": "gz.msgs.Pose_V",
                    "direction": "GZ_TO_ROS",
                },
                {
                    "ros_topic_name": "/cmd_vel",
                    "gz_topic_name": "/cmd_vel",
                    "ros_type_name": "geometry_msgs/msg/Twist",
                    "gz_type_name": "gz.msgs.Twist",
                    "direction": "ROS_TO_GZ",
                },
            ]
            for topic, kind, gz_kind in [
                ("/camera/image", "Image", "Image"),
                ("/camera/depth_image", "Image", "Image"),
                ("/camera/camera_info", "CameraInfo", "CameraInfo"),
            ]:
                bridges.append(
                    {
                        "ros_topic_name": topic,
                        "gz_topic_name": topic,
                        "ros_type_name": "sensor_msgs/msg/" + kind,
                        "gz_type_name": "gz.msgs." + gz_kind,
                        "direction": "GZ_TO_ROS",
                    }
                )
            if scenario == "dynamic":
                for actor in ACTORS:
                    bridges.append(
                        {
                            "ros_topic_name": "/dynamic/" + actor["name"],
                            "gz_topic_name": "/model/" + actor["name"] + "/pose",
                            "ros_type_name": "tf2_msgs/msg/TFMessage",
                            "gz_type_name": "gz.msgs.Pose_V",
                            "direction": "GZ_TO_ROS",
                        }
                    )
            (run / "bridge.yaml").write_text(yaml.safe_dump(bridges))
            start(
                "bridge",
                [
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "--ros-args",
                    "-p",
                    "config_file:=" + str(run / "bridge.yaml"),
                ],
            )
            topics = [
                "/clock",
                "/scan",
                "/lidar3d/points",
                "/wheel_odom",
                "/odom",
                "/truth",
                "/tf",
                "/tf_static",
                "/amcl_pose",
                "/map",
                "/teleop/cmd_vel",
                "/cmd_vel",
                "/camera/image",
                "/camera/depth_image",
                "/camera/camera_info",
                "/localization/markers",
            ]
            topics += (
                ["/dynamic/" + actor["name"] for actor in ACTORS]
                if scenario == "dynamic"
                else []
            )
            start(
                "recorder",
                [
                    "ros2",
                    "bag",
                    "record",
                    "--storage",
                    "sqlite3",
                    "-o",
                    run / "bag",
                    "--topics",
                    *topics,
                ],
            )
            if scenario == "dynamic":
                start(
                    "dynamic_scene",
                    [
                        "python3",
                        "-u",
                        "-m",
                        "slam_physical.dynamic_scene",
                        "--output",
                        run,
                        "--phase",
                        actor_phase,
                    ],
                )
        start(
            "map_server",
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_server",
                "--ros-args",
                "--params-file",
                config,
                "-p",
                "yaml_filename:=" + str(map_file),
            ],
        )
        amcl_process = start(
            "amcl",
            ["ros2", "run", "nav2_amcl", "amcl", "--ros-args", "--params-file", config],
        )
        start(
            "lifecycle",
            [
                "ros2",
                "run",
                "nav2_lifecycle_manager",
                "lifecycle_manager",
                "--ros-args",
                "-r",
                "__node:=lifecycle_manager_localization",
                "--params-file",
                config,
            ],
        )
        if gui:
            start(
                "rviz",
                [
                    "bash",
                    ROOT / "scripts/rviz_localization.sh",
                    run,
                    "-d",
                    ROOT / "rviz/localization.rviz",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
            )
        player = None
        playback_completed = False
        start_wall = time.monotonic()
        last_status = start_wall
        last_resource = start_wall
        previous_cpu = 0.0
        previous_sim = 0.0
        while not any((parent / "STOP").exists() for parent in [run, *run.parents[:3]]):
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_resource >= 1:
                cpu, rss = process_tree_counters(amcl_process.pid)
                sim = node.get_clock().now().nanoseconds * 1e-9
                if previous_cpu:
                    resource_csv.writerow(
                        [
                            now - start_wall,
                            sim,
                            (sim - previous_sim) / (now - last_resource),
                            (cpu - previous_cpu) / (now - last_resource) * 100,
                            rss,
                        ]
                    )
                    resource_file.flush()
                last_resource, previous_cpu, previous_sim = now, cpu, sim
            if replay and player is None and now - start_wall > 5:
                # Exclude old localization results, TF, commands and maps; replay inputs only.
                player = start(
                    "player",
                    [
                        "ros2",
                        "bag",
                        "play",
                        replay / "bag",
                        "--rate",
                        "1.0",
                        "--topics",
                        "/clock",
                        "/scan",
                        "/wheel_odom",
                        "/truth",
                        *(
                            ["/dynamic/" + actor["name"] for actor in ACTORS]
                            if scenario == "dynamic"
                            else []
                        ),
                    ],
                )
            if node.first_ready:
                elapsed = (
                    node.get_clock().now().nanoseconds * 1e-9 - node.first_ready_sim
                )
                if autodrive:
                    message = Twist()
                    message.linear.x, message.angular.z = drive_command(elapsed, route)
                    automation.publish(message)
                if duration and elapsed >= duration:
                    break
            elif now - start_wall > 100:
                raise RuntimeError(
                    "AMCL readiness timed out. Inspect gazebo/bridge/amcl/lifecycle logs."
                )
            if player and player.poll() is not None:
                if player.returncode:
                    raise RuntimeError("Bag replay failed")
                for _ in range(30):
                    rclpy.spin_once(node, timeout_sec=0.02)
                playback_completed = True
                break
            for name, process in children:
                if name not in ("player",) and process.poll() is not None:
                    raise RuntimeError(name + " exited; see " + name + ".log")
            if now - last_status > 5:
                print(
                    f"LIVE: poses={node.amcl_count}, truth_matches={node.matched_count}, LiDAR frames={len(node.clouds)}, RGB-D={len(node.rgb_records)}",
                    flush=True,
                )
                last_status = now
        if replay and not playback_completed:
            raise InterruptedError(
                "Replay ended before the entire held-out recording was consumed"
            )
        status = "completed"
    except BaseException as error:
        status = "failed"
        write_json(
            run / "error.json", {"type": type(error).__name__, "message": str(error)}
        )
        raise
    finally:
        cleanup_error = None
        try:
            node.finish()
        except BaseException as error:
            cleanup_error = error
            status = "failed"
            write_json(
                run / "capture_save_error.json",
                {"type": type(error).__name__, "message": str(error)},
            )
        # SIGINT lets rosbag close its SQLite transaction and metadata cleanly.
        for name, process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for name, process in children:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for handle in handles:
            handle.close()
        resource_file.close()
        node.destroy_node()
        rclpy.try_shutdown()
        summary = localization_report(run)
        metadata = json.loads((run / "session.json").read_text())
        metadata["status"] = status
        write_json(run / "session.json", metadata)
        print("RESULTS: " + str(run), flush=True)
        print(json.dumps(summary, indent=2), flush=True)
        if cleanup_error:
            raise cleanup_error


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map", type=Path)
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--autodrive", action="store_true")
    parser.add_argument("--odom-linear-scale", type=float, default=1.0)
    parser.add_argument("--odom-angular-scale", type=float, default=1.0)
    parser.add_argument("--scenario", choices=["static", "dynamic"], default="static")
    parser.add_argument("--actor-phase", type=float, default=0.0)
    parser.add_argument(
        "--route", choices=["legacy", "mapping", "test"], default="legacy"
    )
    args = parser.parse_args()
    session(
        args.output.resolve(),
        args.map,
        args.replay,
        not args.no_gui,
        args.duration,
        args.autodrive,
        args.odom_linear_scale,
        args.odom_angular_scale,
        args.scenario,
        args.actor_phase,
        args.route,
    )


if __name__ == "__main__":
    main()
