"""Live RViz presentation of maps produced by the current run, via file IPC."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from slam_physical.storage import write_json

__all__ = ["view"]


def view(run: Path) -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    os.environ["ROS_DOMAIN_ID"] = str(
        20 + int(hashlib.sha256(str(run).encode()).hexdigest()[:4], 16) % 60
    )
    rclpy.init()
    node = rclpy.create_node("slam_physical_results")
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "display_origin"
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    names = ["raw", "C0", "C1", "C2", "C4"]
    offsets = {name: index * 15 for index, name in enumerate(names)}
    config = Path(__file__).resolve().parents[2] / "rviz/static.rviz"
    if run.name.startswith("reproduce-"):
        import yaml

        settings = yaml.safe_load(config.read_text())
        camera = settings["Visualization Manager"]["Views"]["Current"]
        camera.update(Distance=25, **{"Focal Point": {"X": 0.0, "Y": 0.0, "Z": 0.0}})
        names = ["C4"]
        offsets = {"C4": 0.0}
        if any(method in run.name for method in ("beautymap", "dufomap")):
            names = ["raw", "C4"]
            deadline = time.monotonic() + 10
            while (
                not (run / "viewer.json").exists()
                and time.monotonic() < deadline
                and not (run / "STOP").exists()
            ):
                time.sleep(0.1)
            state_path = run / "viewer.json"
            if state_path.exists():
                initial = json.loads(state_path.read_text())
                points = np.load(initial["clouds"]["raw"]["path"])
                low, high = points.min(axis=0), points.max(axis=0)
                spacing = float(high[0] - low[0] + 10)
                offsets = {"raw": 0.0, "C4": spacing}
                center = (low + high) / 2
                camera.update(
                    Distance=max(25.0, spacing * 3),
                    **{
                        "Focal Point": {
                            "X": float(center[0] + spacing / 2),
                            "Y": float(center[1]),
                            "Z": float(center[2]),
                        }
                    },
                )
        config = run / "viewer.rviz"
        config.write_text(yaml.safe_dump(settings))
    publishers = {
        name: node.create_publisher(PointCloud2, f"/slam_physical/{name}", qos)
        for name in names
    }
    labels = node.create_publisher(MarkerArray, "/slam_physical/labels", qos)
    gui_log = (run / "rviz.log").open("w")
    gui = subprocess.Popen(
        ["rviz2", "-d", str(config)], stdout=gui_log, stderr=subprocess.STDOUT
    )
    write_json(
        run / "viewer-ready.json",
        {"pid": os.getpid(), "rviz_pid": gui.pid, "mode": "actual_current_run_files"},
    )
    try:
        while not (run / "STOP").exists() and gui.poll() is None:
            state_path = run / "comparison/viewer.json"
            if not state_path.exists():
                state_path = run / "viewer.json"
            if state_path.exists():
                state = json.loads(state_path.read_text())
                messages = []
                for column, name in enumerate(names):
                    record = state.get("clouds", {}).get(name)
                    if not record:
                        continue
                    points = np.load(record["path"], allow_pickle=False).astype("<f4")
                    if len(points) > 120000:
                        points = points[:: int(np.ceil(len(points) / 120000))]
                    points = points.copy()
                    points[:, 0] += offsets[name]
                    message = PointCloud2()
                    message.header = Header(
                        frame_id="map", stamp=node.get_clock().now().to_msg()
                    )
                    message.height, message.width = 1, len(points)
                    message.fields = [
                        PointField(
                            name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1
                        )
                        for i, n in enumerate(("x", "y", "z"))
                    ]
                    message.is_bigendian, message.is_dense = False, True
                    message.point_step, message.row_step = 12, len(points) * 12
                    message.data = points.tobytes()
                    publishers[name].publish(message)
                    marker = Marker()
                    marker.header = message.header
                    marker.ns, marker.id, marker.type, marker.action = (
                        "groups",
                        column,
                        Marker.TEXT_VIEW_FACING,
                        Marker.ADD,
                    )
                    marker.pose.position.x = float(points[:, 0].mean())
                    marker.pose.position.y = float(points[:, 1].mean())
                    marker.pose.position.z = float(points[:, 2].max() + 1.0)
                    marker.pose.orientation.w = 1.0
                    marker.scale.z = (
                        2.0
                        if run.name.startswith(
                            ("reproduce-beautymap", "reproduce-dufomap")
                        )
                        else 0.7
                    )
                    marker.color.r = marker.color.g = marker.color.b = (
                        marker.color.a
                    ) = 1.0
                    marker.text = name + " | " + str(record["points"]) + " points"
                    messages.append(marker)
                labels.publish(MarkerArray(markers=messages))
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.4)
    finally:
        if gui.poll() is None:
            gui.terminate()
            gui.wait(timeout=10)
        gui_log.close()
        node.destroy_node()
        rclpy.shutdown()
