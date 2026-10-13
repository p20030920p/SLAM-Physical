"""Actual Gazebo GPU-LiDAR acquisition; pose telemetry is a required gate."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

import numpy as np

from slam_physical.execution import check_stop
from slam_physical.geometry import surface_distance
from slam_physical.storage import digest, publish_cloud, write_json
from slam_physical.world import build_world, departure_box, sensor_poses

__all__ = ["capture"]


def _cloud_xyz(message):
    endian = ">" if message.is_bigendian else "<"
    fields = {field.name: field.offset for field in message.fields}
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": [fields[name] for name in ("x", "y", "z")],
            "itemsize": message.point_step,
        }
    )
    raw = np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=bytes(message.data),
        strides=(message.row_step, message.point_step),
    )
    points = np.column_stack([raw[name].ravel() for name in ("x", "y", "z")])
    valid = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points, axis=1)
    return points[valid & (ranges > 0.3) & (ranges < 11.99)].copy()


def capture(
    output: Path,
    assets: Path,
    count: int = 40,
    gui: bool = False,
    scenario: str = "static",
) -> dict:
    import rclpy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage

    output.mkdir(parents=True, exist_ok=False)
    if count < 1 or scenario not in ("static", "departure"):
        raise ValueError("Require positive frame count and a supported scenario")
    boxes = build_world(output / "world.sdf", assets, scenario)
    token = uuid.uuid4().hex[:10]
    os.environ["GZ_PARTITION"] = f"slam_physical_{token}"
    os.environ["ROS_DOMAIN_ID"] = str(100 + int(token[:2], 16) % 100)
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(assets / "src/race_gazebo/models")
    os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    os.environ.setdefault("MESA_GL_VERSION_OVERRIDE", "4.5")
    command = ["gz", "sim", "-r"]
    if not gui:
        command += ["-s", "--headless-rendering"]
    command += [str(output / "world.sdf")]
    logs = []
    processes = []
    rclpy.init()
    node = rclpy.create_node("slam_physical_capture")
    state = {
        "clock": 0.0,
        "scan": None,
        "scan_clock": -1.0,
        "pose": None,
        "pose_ids": [],
        "actor": None,
    }

    def on_scan(message):
        state["scan"] = message
        state["scan_clock"] = (
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        )

    def on_clock(message):
        state["clock"] = message.clock.sec + message.clock.nanosec * 1e-9

    def on_pose(message):
        state["pose_ids"] = [item.child_frame_id for item in message.transforms]
        # The bridge drops Pose.name. Use a dedicated model-only publisher,
        # require exactly one transform, and verify its independently observed pose.
        if len(message.transforms) == 1:
            item = message.transforms[0]
            t, q = item.transform.translation, item.transform.rotation
            state["pose"] = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w])

    def on_actor(message):
        if len(message.transforms) == 1:
            t = message.transforms[0].transform.translation
            state["actor"] = np.array([t.x, t.y, t.z])

    subscriptions = [
        node.create_subscription(
            PointCloud2, "/slam_physical/lidar/points", on_scan, 5
        ),
        node.create_subscription(Clock, "/clock", on_clock, 5),
        node.create_subscription(TFMessage, "/slam_physical/poses", on_pose, 5),
    ]
    if scenario == "departure":
        subscriptions.append(
            node.create_subscription(TFMessage, "/slam_physical/actor", on_actor, 5)
        )
    bridge = [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        "/slam_physical/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/model/scan_rig/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    ]
    if scenario == "departure":
        bridge.append("/model/moving_target/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V")
    bridge += ["--ros-args", "-r", "/model/scan_rig/pose:=/slam_physical/poses"]
    if scenario == "departure":
        bridge += ["-r", "/model/moving_target/pose:=/slam_physical/actor"]
    try:
        for name, arguments in [
            ("gazebo", command),
            ("bridge", bridge),
        ]:
            log = (output / f"{name}.log").open("w")
            logs.append(log)
            print("START:", " ".join(arguments), flush=True)
            processes.append(
                subprocess.Popen(
                    arguments,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        poses = sensor_poses(count)
        collected = {}
        telemetry = []
        for index, pose in enumerate(poses):
            check_stop(output)
            yaw = math.atan2(pose[1, 0], pose[0, 0])
            desired = [*pose[:3, 3], 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
            request = (
                f'name: "scan_rig", position: {{x: {desired[0]}, y: {desired[1]}, z: {desired[2]}}}, '
                f"orientation: {{z: {desired[5]}, w: {desired[6]}}}"
            )
            deadline = time.monotonic() + 100
            actor = departure_box(index) if scenario == "departure" else None
            if actor is not None:
                x, y, z = actor["center"]
                actor_request = f'name: "moving_target", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}'
                while True:
                    check_stop(output)
                    result = subprocess.run(
                        [
                            "gz",
                            "service",
                            "-s",
                            "/world/slam_static/set_pose",
                            "--reqtype",
                            "gz.msgs.Pose",
                            "--reptype",
                            "gz.msgs.Boolean",
                            "--timeout",
                            "3000",
                            "--req",
                            actor_request,
                        ],
                        capture_output=True,
                        text=True,
                        timeout=8,
                    )
                    if result.returncode == 0 and "true" in result.stdout:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError("Actor set_pose service unavailable")
                    rclpy.spin_once(node, timeout_sec=0.2)
            while True:
                check_stop(output)
                result = subprocess.run(
                    [
                        "gz",
                        "service",
                        "-s",
                        "/world/slam_static/set_pose",
                        "--reqtype",
                        "gz.msgs.Pose",
                        "--reptype",
                        "gz.msgs.Boolean",
                        "--timeout",
                        "3000",
                        "--req",
                        request,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=8,
                )
                if result.returncode == 0 and "true" in result.stdout:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        "Gazebo set_pose service unavailable: "
                        + result.stdout[-500:]
                        + result.stderr[-500:]
                    )
                rclpy.spin_once(node, timeout_sec=0.2)
            # Require a new sensor frame AND independently published matching pose.
            after = state["clock"] + 0.6
            while True:
                check_stop(output)
                rclpy.spin_once(node, timeout_sec=0.1)
                observed = state["pose"]
                pose_ok = observed is not None and np.allclose(
                    observed[:3], desired[:3], atol=1e-5
                )
                pose_ok = (
                    pose_ok
                    and min(
                        np.linalg.norm(observed[3:] - desired[3:]),
                        np.linalg.norm(observed[3:] + desired[3:]),
                    )
                    < 1e-5
                )
                actor_ok = actor is None or (
                    state["actor"] is not None
                    and np.allclose(state["actor"], actor["center"], atol=1e-5)
                )
                if (
                    pose_ok
                    and actor_ok
                    and state["scan_clock"] > after
                    and state["scan"] is not None
                ):
                    points = _cloud_xyz(state["scan"])
                    if len(points) >= 500:
                        break
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"Sensor/pose gate failed: clock={state['clock']} scan={state['scan_clock']} pose={observed} ids={state['pose_ids'][:15]}"
                    )
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError("Gazebo or bridge exited; inspect capture logs")
            collected[f"scan_{index:04d}"] = points.astype(np.float32)
            global_points = points @ pose[:3, :3].T + pose[:3, 3]
            static_distance = surface_distance(global_points, boxes)
            dynamic_distance = (
                surface_distance(global_points, [actor])
                if actor
                else np.full(len(points), np.inf)
            )
            labels = np.full(len(points), -1, dtype=np.int8)
            labels[(static_distance <= 0.04) & (dynamic_distance > 0.04)] = 0
            labels[(dynamic_distance <= 0.04) & (static_distance > 0.04)] = 1
            collected[f"labels_{index:04d}"] = labels
            item = {
                "observation_id": index,
                "sensor_time": state["scan_clock"],
                "observed_pose_xyzw": observed.tolist(),
                "static_points": int((labels == 0).sum()),
                "dynamic_points": int((labels == 1).sum()),
                "unclassified_points": int((labels < 0).sum()),
            }
            if actor:
                item.update(
                    actor_box=actor, observed_actor_position=state["actor"].tolist()
                )
            telemetry.append(item)
            print(
                f"CAPTURE {index+1}/{count}: {len(points)} actual LiDAR points; pose telemetry verified",
                flush=True,
            )
            # Current capture is shown as a genuine newly acquired frame.
            publish_cloud(
                output.parent,
                "raw",
                points @ pose[:3, :3].T + pose[:3, 3],
                f"Gazebo capture {index+1}/{count}",
            )
        collected["poses"] = poses
        np.savez_compressed(output / "scans.npz", **collected)
        metadata = {
            "backend": "gazebo-harmonic",
            "frames": count,
            "pose_telemetry_verified": True,
            "pose_telemetry": telemetry,
            "static_boxes": boxes,
            "world_sha256": digest(output / "world.sdf"),
            "scans_sha256": digest(output / "scans.npz"),
            "sim2real_commit": "53324d40def0dd753b4a99021b8fe596955a1ecd",
            "scenario": scenario,
            "label_rule": "Independent SDF surface distance <=4 cm; ambiguous/unknown excluded; labels never sent to native code",
            "scope": "Measurement annex in Sim2Real Gazebo world. Kinematic sensor/actor steps, no Nav2 or real loop closure.",
            "status": "captured",
        }
        write_json(output / "capture.json", metadata)
        return metadata
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
