"""Wheel odometry relay, evaluation-only truth, live errors and mapping inputs."""

from __future__ import annotations

import csv
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from slam_physical.capture import _cloud_xyz
from slam_physical.pose_tools import interpolate_pose, matrix, wrap, yaw_of
from slam_physical.storage import write_json

__all__ = ["DriveTelemetry"]


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class DriveTelemetry(Node):
    def __init__(self, run: Path, linear_scale=1.0, angular_scale=1.0, collect=True):
        super().__init__(
            "drive_telemetry",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.run, self.collect = run, collect
        self.scales = linear_scale, angular_scale
        self.truth, self.odometry = deque(maxlen=1500), deque(maxlen=1500)
        self.scan_arrivals = {}
        self.map_odom = None
        self.last_amcl_wall = None
        self.last_command = None
        self.last_command_time = 0.0
        self.last_wheel = None
        self.odom_pose = np.zeros(3)
        self.clouds, self.cloud_poses = [], []
        self.cloud_stamps = []
        self.last_collection = -1e9
        self.rgb, self.depth, self.camera = None, None, None
        self.rgb_records = []
        self.last_rgb_time = -1e9
        self.amcl_count = 0
        self.matched_count = 0
        self.first_ready = None
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        static = []
        for name, xyz, quat in [
            ("base_link", (0, 0, 0.27), (0, 0, 0, 1)),
            ("lidar_link", (0, 0, 0.65), (0, 0, 0, 1)),
            ("camera_optical_frame", (0.2, 0, 0.75), (-0.5, 0.5, -0.5, 0.5)),
        ]:
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = "base_footprint", name
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ) = map(float, xyz)
            q = transform.transform.rotation
            q.x, q.y, q.z, q.w = map(float, quat)
            static.append(transform)
        self.static.sendTransform(static)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.markers = self.create_publisher(MarkerArray, "/localization/markers", 10)
        self.truth_pub = self.create_publisher(RosPath, "/localization/truth_path", 10)
        self.estimate_pub = self.create_publisher(
            RosPath, "/localization/estimate_path", 10
        )
        self.truth_path, self.estimate_path = RosPath(), RosPath()
        self.truth_path.header.frame_id = self.estimate_path.header.frame_id = "map"
        self.amcl_file = (run / "amcl_samples.csv").open("w", newline="")
        self.amcl_csv = csv.writer(self.amcl_file)
        self.amcl_csv.writerow(
            [
                "stamp_sim_s",
                "truth_x",
                "truth_y",
                "estimate_x",
                "estimate_y",
                "xy_error_m",
                "yaw_error_rad",
                "truth_sync_distance_s",
                "scan_to_pose_observed_wall_ms",
                "pose_gap_wall_s",
                "pose_age_sim_s",
            ]
        )
        self.live_file = (run / "live_error.csv").open("w", newline="")
        self.live_csv = csv.writer(self.live_file)
        self.live_csv.writerow(
            [
                "stamp_sim_s",
                "truth_x",
                "truth_y",
                "estimate_x",
                "estimate_y",
                "xy_error_m",
                "yaw_error_rad",
                "amcl_age_wall_s",
            ]
        )
        self.owned_subscriptions = [
            self.create_subscription(Odometry, "/wheel_odom", self.on_wheel, 20),
            self.create_subscription(
                TFMessage, "/truth", self.on_truth, qos_profile_sensor_data
            ),
            self.create_subscription(TFMessage, "/tf", self.on_tf, 50),
            self.create_subscription(Twist, "/teleop/cmd_vel", self.on_command, 10),
            self.create_subscription(
                LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
            ),
            self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 20
            ),
        ]
        if collect:
            self.owned_subscriptions += [
                self.create_subscription(
                    PointCloud2,
                    "/lidar3d/points",
                    self.on_cloud,
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    Image, "/camera/image", self.on_rgb, qos_profile_sensor_data
                ),
                self.create_subscription(
                    Image, "/camera/depth_image", self.on_depth, qos_profile_sensor_data
                ),
                self.create_subscription(
                    CameraInfo,
                    "/camera/camera_info",
                    self.on_camera,
                    qos_profile_sensor_data,
                ),
            ]
        self.create_timer(0.1, self.tick)

    def on_command(self, message):
        self.last_command, self.last_command_time = message, time.monotonic()

    def on_scan(self, message):
        self.scan_arrivals[round(seconds(message.header.stamp), 6)] = time.monotonic()
        if len(self.scan_arrivals) > 300:
            del self.scan_arrivals[min(self.scan_arrivals)]

    def on_tf(self, message):
        for item in message.transforms:
            if item.header.frame_id == "map" and item.child_frame_id == "odom":
                t = item.transform.translation
                self.map_odom = matrix(t.x, t.y, yaw_of(item.transform.rotation))

    def on_wheel(self, message):
        pose = message.pose.pose
        raw = np.array([pose.position.x, pose.position.y, yaw_of(pose.orientation)])
        if self.last_wheel is None:
            self.odom_pose = raw.copy()
        else:
            delta = raw[:2] - self.last_wheel[:2]
            angle = self.odom_pose[2] - self.last_wheel[2]
            rotation = np.array(
                [
                    [math.cos(angle), -math.sin(angle)],
                    [math.sin(angle), math.cos(angle)],
                ]
            )
            self.odom_pose[:2] += self.scales[0] * rotation @ delta
            self.odom_pose[2] = wrap(
                self.odom_pose[2] + self.scales[1] * wrap(raw[2] - self.last_wheel[2])
            )
        self.last_wheel = raw
        x, y, yaw = self.odom_pose
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), 0.0
        pose.orientation.x = pose.orientation.y = 0.0
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        message.header.frame_id, message.child_frame_id = "odom", "base_footprint"
        self.odom_pub.publish(message)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = message.header, "base_footprint"
        transform.transform.translation.x, transform.transform.translation.y = (
            float(x),
            float(y),
        )
        transform.transform.rotation = pose.orientation
        self.tf.sendTransform(transform)
        self.odometry.append(
            (seconds(message.header.stamp), float(x), float(y), float(yaw))
        )

    def on_truth(self, message):
        if len(message.transforms) != 1:
            raise RuntimeError(
                "Dedicated model truth must contain exactly one transform"
            )
        item = message.transforms[0]
        t = item.transform.translation
        self.truth.append(
            (seconds(item.header.stamp), t.x, t.y, yaw_of(item.transform.rotation))
        )

    def on_amcl(self, message):
        wall = time.monotonic()
        stamp = seconds(message.header.stamp)
        self.amcl_count += 1
        match = interpolate_pose(list(self.truth), stamp)
        scan_time = self.scan_arrivals.get(round(stamp, 6))
        gap = wall - self.last_amcl_wall if self.last_amcl_wall else None
        self.last_amcl_wall = wall
        if match:
            truth, sync = match
            pose = message.pose.pose
            error = math.hypot(pose.position.x - truth[0], pose.position.y - truth[1])
            self.amcl_csv.writerow(
                [
                    stamp,
                    truth[0],
                    truth[1],
                    pose.position.x,
                    pose.position.y,
                    error,
                    wrap(yaw_of(pose.orientation) - truth[2]),
                    sync,
                    (wall - scan_time) * 1000 if scan_time else None,
                    gap,
                    self.get_clock().now().nanoseconds * 1e-9 - stamp,
                ]
            )
            self.amcl_file.flush()
            self.matched_count += 1
        if self.first_ready is None and self.truth and self.odometry:
            self.first_ready = wall
            write_json(
                self.run / "ready.json",
                {
                    "status": "AMCL_READY",
                    "scan_received": bool(self.scan_arrivals),
                    "truth_received": True,
                    "wheel_odometry_received": True,
                },
            )
            print(
                "AMCL_READY: keyboard may drive; automatic recording is active.",
                flush=True,
            )

    def on_cloud(self, message):
        stamp = seconds(message.header.stamp)
        if stamp - self.last_collection < 0.9 or len(self.clouds) >= 160:
            return
        match = interpolate_pose(list(self.odometry), stamp)
        if not match:
            return
        x, y, yaw = match[0]
        pose = matrix(x + 6, y + 6, yaw, 0.65)
        if (
            self.cloud_poses
            and np.linalg.norm(pose[:2, 3] - self.cloud_poses[-1][:2, 3]) < 0.08
            and abs(
                wrap(
                    yaw
                    - math.atan2(self.cloud_poses[-1][1, 0], self.cloud_poses[-1][0, 0])
                )
            )
            < 0.08
        ):
            return
        self.clouds.append(_cloud_xyz(message).astype(np.float32))
        self.cloud_poses.append(pose)
        self.cloud_stamps.append(stamp)
        self.last_collection = stamp

    def on_camera(self, message):
        self.camera = message

    def on_rgb(self, message):
        self.rgb = message
        self.collect_rgbd()

    def on_depth(self, message):
        self.depth = message
        self.collect_rgbd()

    def collect_rgbd(self):
        if (
            self.rgb is None
            or self.depth is None
            or self.camera is None
            or len(self.rgb_records) >= 80
        ):
            return
        stamp = seconds(self.rgb.header.stamp)
        if (
            stamp - self.last_rgb_time < 0.9
            or abs(stamp - seconds(self.depth.header.stamp)) > 0.005
        ):
            return
        match = interpolate_pose(list(self.odometry), stamp)
        if not match:
            return
        from PIL import Image as Pillow

        x, y, yaw = match[0]
        optical = matrix(x + 6, y + 6, yaw) @ matrix(0.2, 0, 0, 0.75)
        if self.rgb_records:
            previous = np.array(self.rgb_records[-1]["pose"])
            previous_yaw = math.atan2(previous[1, 2], previous[0, 2])
            if (
                np.linalg.norm(optical[:2, 3] - previous[:2, 3]) < 0.08
                and abs(wrap(yaw - previous_yaw)) < 0.08
            ):
                return

        folder = self.run / "rgbd/room0/results"
        folder.mkdir(parents=True, exist_ok=True)
        index = len(self.rgb_records)
        rgb = np.ndarray(
            (self.rgb.height, self.rgb.width, 3),
            dtype=np.uint8,
            buffer=bytes(self.rgb.data),
            strides=(self.rgb.step, 3, 1),
        )
        if self.rgb.encoding == "bgr8":
            rgb = rgb[:, :, ::-1]
        if self.depth.encoding not in ("32FC1", "32FC"):
            raise ValueError("Expected metric 32FC1 Gazebo depth")
        depth = np.ndarray(
            (self.depth.height, self.depth.width),
            dtype="<f4",
            buffer=bytes(self.depth.data),
            strides=(self.depth.step, 4),
        )
        depth = np.where(
            np.isfinite(depth) & (depth > 0) & (depth < 12), depth * 1000, 0
        ).astype(np.uint16)
        Pillow.fromarray(rgb).save(folder / f"frame{index:06d}.jpg")
        Pillow.fromarray(depth).save(folder / f"depth{index:06d}.png")
        optical[:3, :3] = optical[:3, :3] @ np.array(
            [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        )
        self.rgb_records.append({"stamp": stamp, "pose": optical.tolist()})
        self.last_rgb_time = stamp

    def tick(self):
        # Standard keyboard publishes on key presses. A held key repeats; stale input stops.
        command = (
            self.last_command
            if time.monotonic() - self.last_command_time < 0.6
            else Twist()
        )
        self.cmd_pub.publish(command)
        if not self.truth or not self.odometry or self.map_odom is None:
            return
        stamp, x, y, yaw = self.odometry[-1]
        match = interpolate_pose(list(self.truth), stamp)
        if not match:
            return
        truth = match[0]
        estimated = self.map_odom @ matrix(x, y, yaw)
        estimate = np.array(
            [
                estimated[0, 3],
                estimated[1, 3],
                math.atan2(estimated[1, 0], estimated[0, 0]),
            ]
        )
        age = time.monotonic() - self.last_amcl_wall if self.last_amcl_wall else None
        error = float(np.linalg.norm(estimate[:2] - truth[:2]))
        self.live_csv.writerow(
            [stamp, *truth[:2], *estimate[:2], error, wrap(estimate[2] - truth[2]), age]
        )
        self.live_file.flush()
        markers = []
        for index, (name, pose, color) in enumerate(
            [("Truth", truth, (0.2, 1.0, 0.3)), ("AMCL", estimate, (0.2, 0.6, 1.0))]
        ):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = (
                "map",
                self.get_clock().now().to_msg(),
            )
            marker.ns, marker.id, marker.type, marker.action = (
                "robot",
                index,
                Marker.ARROW,
                Marker.ADD,
            )
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                float(pose[0]),
                float(pose[1]),
                0.2,
            )
            marker.pose.orientation.z, marker.pose.orientation.w = (
                math.sin(pose[2] / 2),
                math.cos(pose[2] / 2),
            )
            marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.12, 0.12
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            markers.append(marker)
            path = self.truth_path if index == 0 else self.estimate_path
            item = PoseStamped()
            item.header, item.pose = marker.header, marker.pose
            path.header.stamp = marker.header.stamp
            path.poses.append(item)
            path.poses = path.poses[-6000:]
        self.truth_pub.publish(self.truth_path)
        self.estimate_pub.publish(self.estimate_path)
        text = Marker()
        text.header = markers[0].header
        text.ns, text.id, text.type, text.action = (
            "error",
            0,
            Marker.TEXT_VIEW_FACING,
            Marker.ADD,
        )
        text.pose.position.x, text.pose.position.y, text.pose.position.z = (
            6.0,
            12.65,
            0.3,
        )
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"Error {error*100:.1f} cm | yaw {math.degrees(wrap(estimate[2]-truth[2])):.1f} deg\nGreen: truth | Blue: AMCL | Last pose: {age:.2f} s"
        markers.append(text)
        self.markers.publish(MarkerArray(markers=markers))

    def finish(self):
        if rclpy.ok():
            self.cmd_pub.publish(Twist())
        self.amcl_file.close()
        self.live_file.close()
        if self.clouds:
            archive = {f"scan_{i:04d}": scan for i, scan in enumerate(self.clouds)}
            archive.update(
                poses=np.array(self.cloud_poses), stamps=np.array(self.cloud_stamps)
            )
            np.savez_compressed(self.run / "mapping_scans.npz", **archive)
        if self.rgb_records:
            folder = self.run / "rgbd"
            (folder / "room0/traj.txt").write_text(
                "".join(
                    " ".join(map(str, np.array(item["pose"]).ravel())) + "\n"
                    for item in self.rgb_records
                )
            )
            k = self.camera.k
            write_json(
                folder / "cam_params.json",
                {
                    "camera": {
                        "w": self.camera.width,
                        "h": self.camera.height,
                        "fx": k[0],
                        "fy": k[4],
                        "cx": k[2],
                        "cy": k[5],
                        "scale": 1000.0,
                    }
                },
            )
            write_json(
                folder / "capture.json",
                {
                    "poses": "wheel odometry + declared start anchor + fixed optical extrinsic; no truth poses",
                    "frames": len(self.rgb_records),
                },
            )
        write_json(
            self.run / "telemetry.json",
            {
                "amcl_callbacks": self.amcl_count,
                "truth_matched_amcl_samples": self.matched_count,
                "mapping_scans": len(self.clouds),
                "rgbd_frames": len(self.rgb_records),
                "odom_scales": self.scales,
                "truth_usage": "evaluation/display only; never a localization or map-building input",
            },
        )
