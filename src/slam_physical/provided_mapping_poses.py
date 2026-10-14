"""Export simulator-provided training poses for a controlled mapping comparison.

This explicitly uses simulator truth for mapping A only. It is never invoked on
the held-out test by the benchmark, and AMCL never subscribes to these files.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import sqlite3
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation, Slerp

from slam_physical.pose_tools import matrix, wrap, yaw_of
from slam_physical.storage import digest, write_json


def interpolate_transforms(stamps, positions, quaternions, requested):
    requested = np.asarray(requested, dtype=float)
    if requested.min() < stamps[0] or requested.max() > stamps[-1]:
        raise ValueError("Provided poses must not extrapolate beyond recorded truth")
    transforms = np.repeat(np.eye(4)[None], len(requested), axis=0)
    transforms[:, :3, :3] = Slerp(stamps, Rotation.from_quat(quaternions))(
        requested
    ).as_matrix()
    for axis in range(3):
        transforms[:, axis, 3] = np.interp(requested, stamps, positions[:, axis])
    return transforms


def provide(source: Path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    metadata = json.loads((source / "session.json").read_text())
    if metadata.get("route") != "mapping":
        raise ValueError("Provided pose export is restricted to mapping A recordings")
    saved = sorted((source / "rgbd/room0/results").glob("frame*.jpg"))
    candidates = {hashlib.sha256(path.read_bytes()).hexdigest(): [] for path in saved}
    truth, wheels = [], []
    last_wheel = None
    odom_pose = np.zeros(3)
    linear, angular = metadata["odom_linear_scale"], metadata["odom_angular_scale"]
    for bag in sorted((source / "bag").glob("*.db3")):
        with sqlite3.connect(bag) as db:
            topics = {
                name: get_message(kind)
                for name, kind in db.execute("SELECT name,type FROM topics")
                if name in ("/truth", "/wheel_odom", "/camera/image")
            }
            query = "SELECT topics.name,messages.data FROM messages JOIN topics ON messages.topic_id=topics.id WHERE topics.name IN ('/truth','/wheel_odom','/camera/image') ORDER BY messages.timestamp"
            for topic, data in db.execute(query):
                message = deserialize_message(data, topics[topic])
                if topic == "/truth":
                    item = message.transforms[0]
                    p, q = item.transform.translation, item.transform.rotation
                    stamp = item.header.stamp.sec + item.header.stamp.nanosec * 1e-9
                    truth.append((stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
                    continue
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                if topic == "/wheel_odom":
                    p, q = message.pose.pose.position, message.pose.pose.orientation
                    raw = np.array([p.x, p.y, yaw_of(q)])
                    if last_wheel is None:
                        odom_pose = raw.copy()
                    else:
                        delta = raw[:2] - last_wheel[:2]
                        angle = odom_pose[2] - last_wheel[2]
                        odom_pose[:2] += linear * matrix(0, 0, angle)[:2, :2] @ delta
                        odom_pose[2] = wrap(
                            odom_pose[2] + angular * wrap(raw[2] - last_wheel[2])
                        )
                    last_wheel = raw
                    wheels.append(
                        [stamp, odom_pose[0] + 6, odom_pose[1] + 6, odom_pose[2]]
                    )
                else:
                    image = np.ndarray(
                        (message.height, message.width, 3),
                        dtype=np.uint8,
                        buffer=bytes(message.data),
                        strides=(message.step, 3, 1),
                    )
                    if message.encoding == "bgr8":
                        image = image[:, :, ::-1]
                    buffer = io.BytesIO()
                    Image.fromarray(image).save(buffer, format="JPEG")
                    key = hashlib.sha256(buffer.getvalue()).hexdigest()
                    if key in candidates:
                        candidates[key].append(stamp)
    # Header times of saved RGB keyframes are recovered by exact JPEG byte
    # matches. A repeated stationary image is disambiguated using its original
    # wheel-odometry pose; no estimated truth pose is used to select timestamps.
    wheels = np.asarray(wheels)
    camera_poses = np.loadtxt(source / "rgbd/room0/traj.txt", ndmin=2).reshape(-1, 4, 4)
    rgb_stamps, previous = [], -1.0
    for path, pose in zip(saved, camera_poses):
        matches = [
            stamp
            for stamp in candidates[hashlib.sha256(path.read_bytes()).hexdigest()]
            if stamp > previous
        ]
        if not matches:
            raise ValueError(
                "Saved RGB image has no exact recorded message match: " + str(path)
            )
        yaw = np.arctan2(pose[1, 2], pose[0, 2])
        desired = pose[:2, 3] - 0.2 * np.array([np.cos(yaw), np.sin(yaw)])

        def score(stamp):
            xy = np.array(
                [np.interp(stamp, wheels[:, 0], wheels[:, axis]) for axis in (1, 2)]
            )
            heading = np.interp(stamp, wheels[:, 0], np.unwrap(wheels[:, 3]))
            return np.linalg.norm(xy - desired) + abs(wrap(heading - yaw))

        chosen = min(matches, key=score)
        if score(chosen) > 0.03:
            raise ValueError("RGB header/odometry correspondence exceeds tolerance")
        rgb_stamps.append(chosen)
        previous = chosen
    unique = {row[0]: row for row in truth}
    truth = [unique[stamp] for stamp in sorted(unique)]
    stamps = np.array([row[0] for row in truth])
    positions = np.array([row[1] for row in truth])
    quaternions = np.array([row[2] for row in truth])
    lidar_stamps = np.load(source / "mapping_scans.npz", allow_pickle=False)["stamps"]
    lidar_valid = (lidar_stamps >= stamps[0]) & (lidar_stamps <= stamps[-1])
    rgb_stamps = np.asarray(rgb_stamps)
    rgb_valid = (rgb_stamps >= stamps[0]) & (rgb_stamps <= stamps[-1])
    if lidar_valid.sum() < 2 or rgb_valid.sum() < 2:
        raise ValueError("Insufficient training frames within recorded truth coverage")
    lidar = np.full((len(lidar_stamps), 4, 4), np.nan)
    lidar[lidar_valid] = interpolate_transforms(
        stamps, positions, quaternions, lidar_stamps[lidar_valid]
    ) @ matrix(0, 0, 0, 0.5)
    optical = matrix(0.2, 0, 0, 0.6)
    optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    rgbd = np.full((len(rgb_stamps), 4, 4), np.nan)
    rgbd[rgb_valid] = (
        interpolate_transforms(stamps, positions, quaternions, rgb_stamps[rgb_valid])
        @ optical
    )
    np.save(source / "provided_lidar_poses.npy", lidar)
    np.save(source / "provided_rgbd_poses.npy", rgbd)
    write_json(
        source / "provided_pose_record.json",
        {
            "source": "time-interpolated simulator model truth, supplied to mapping A only",
            "scope": "controlled map construction; not a SLAM pose estimate; held-out AMCL receives no truth poses",
            "lidar_frames": int(lidar_valid.sum()),
            "rgbd_frames": int(rgb_valid.sum()),
            "lidar_slots_total": len(lidar),
            "rgbd_slots_total": len(rgbd),
            "rgbd_header_stamps_sim_s": rgb_stamps.tolist(),
            "excluded_lidar_ids_outside_truth_coverage": np.flatnonzero(
                ~lidar_valid
            ).tolist(),
            "excluded_rgbd_ids_outside_truth_coverage": np.flatnonzero(
                ~rgb_valid
            ).tolist(),
            "truth_time_range_sim_s": [float(stamps[0]), float(stamps[-1])],
            "rgbd_stamp_match": "exact saved JPEG bytes + recorded wheel-odometry correspondence",
            "truth_max_sample_gap_s": float(np.diff(stamps).max()),
            "lidar_pose_sha256": digest(source / "provided_lidar_poses.npy"),
            "rgbd_pose_sha256": digest(source / "provided_rgbd_poses.npy"),
            "bag_sha256": {
                bag.name: digest(bag) for bag in (source / "bag").glob("*.db3")
            },
        },
    )
    print(
        f"PROVIDED MAPPING POSES: {lidar_valid.sum()} valid LiDAR, {rgb_valid.sum()} valid RGB-D; unsupported timestamps excluded",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    provide(parser.parse_args().source.resolve())


if __name__ == "__main__":
    main()
