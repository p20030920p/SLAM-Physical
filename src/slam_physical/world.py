"""A static measurement annex beside the pinned Sim2Real-AlgoBench arena."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from slam_physical.storage import write_json

__all__ = ["build_world", "sensor_poses", "departure_box"]


def departure_box(index: int) -> dict:
    return {
        "name": "moving_target",
        "center": [9 + max(0, index - 3) * 0.55, 4, 0.6],
        "size": [0.6, 0.8, 1.2],
    }


def sensor_poses(count: int) -> np.ndarray:
    poses = []
    for i in range(count):
        angle = 2 * math.pi * i / max(count - 1, 1)
        pose = np.eye(4)
        pose[:3, :3] = [
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1],
        ]
        pose[:3, 3] = [6 + 0.8 * math.cos(angle), 6 + 0.8 * math.sin(angle), 1.0]
        poses.append(pose)
    return np.asarray(poses)


def _text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def build_world(path: Path, assets: Path, scenario: str = "static") -> list[dict]:
    model = assets / "src/race_gazebo/models/competition_map_dae/model.sdf"
    if not model.is_file():
        raise FileNotFoundError("Run scripts/fetch_sim2real.sh first")
    root = ET.Element("sdf", version="1.9")
    world = ET.SubElement(root, "world", name="slam_static")
    physics = ET.SubElement(world, "physics", name="fixed", type="ignored")
    _text(physics, "max_step_size", 0.01)
    _text(physics, "real_time_factor", 1)
    for name, class_name in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("sensors", "Sensors"),
    ]:
        plugin = ET.SubElement(
            world,
            "plugin",
            filename=f"gz-sim-{name}-system",
            name=f"gz::sim::systems::{class_name}",
        )
        if name == "sensors":
            _text(plugin, "render_engine", "ogre2")
    light = ET.SubElement(world, "light", name="sun", type="directional")
    _text(light, "pose", "6 6 10 0 0 0")
    _text(light, "diffuse", ".8 .8 .8 1")
    _text(light, "direction", "-.3 -.4 -1")
    include = ET.SubElement(world, "include")
    _text(include, "uri", "model://competition_map_dae")
    _text(include, "pose", "-24 0 0 0 0 0")
    boxes = [
        {"name": "floor", "center": [6, 6, -0.1], "size": [12, 12, 0.2]},
        {"name": "west_wall", "center": [0, 6, 1.5], "size": [0.2, 12, 3]},
        {"name": "east_wall", "center": [12, 6, 1.5], "size": [0.2, 12, 3]},
        {"name": "south_wall", "center": [6, 0, 1.5], "size": [12, 0.2, 3]},
        {"name": "north_wall", "center": [6, 12, 1.5], "size": [12, 0.2, 3]},
        {"name": "static_pillar", "center": [3.5, 4.0, 0.8], "size": [0.6, 0.6, 1.6]},
        {"name": "static_table", "center": [8.7, 7.5, 0.65], "size": [1.2, 1.8, 1.3]},
        {"name": "static_shelf", "center": [3.0, 9.0, 1.0], "size": [0.5, 2.0, 2.0]},
    ]
    if scenario == "departure":
        boxes = [box for box in boxes if box["name"] != "east_wall"]
        boxes += [
            {"name": "east_wall_south", "center": [12, 1.5, 1.5], "size": [0.2, 3, 3]},
            {"name": "east_wall_north", "center": [12, 8.5, 1.5], "size": [0.2, 7, 3]},
        ]
    rendered_boxes = boxes + ([departure_box(0)] if scenario == "departure" else [])
    for box in rendered_boxes:
        element = ET.SubElement(world, "model", name=box["name"])
        _text(element, "static", "true")
        _text(element, "pose", " ".join(map(str, box["center"])) + " 0 0 0")
        if box["name"] == "moving_target":
            publisher = ET.SubElement(
                element,
                "plugin",
                filename="gz-sim-pose-publisher-system",
                name="gz::sim::systems::PosePublisher",
            )
            for tag, value in [
                ("publish_model_pose", "true"),
                ("publish_link_pose", "false"),
                ("publish_nested_model_pose", "false"),
                ("use_pose_vector_msg", "true"),
                ("static_publisher", "false"),
                ("update_frequency", 10),
            ]:
                _text(publisher, tag, value)
        link = ET.SubElement(element, "link", name="body")
        for kind in ("visual", "collision"):
            item = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(item, "geometry")
            shape = ET.SubElement(geometry, "box")
            _text(shape, "size", " ".join(map(str, box["size"])))
            if kind == "visual":
                material = ET.SubElement(item, "material")
                color = ".65 .68 .72 1" if "wall" in box["name"] else ".25 .5 .65 1"
                _text(material, "ambient", color)
                _text(material, "diffuse", color)
    rig = ET.SubElement(world, "model", name="scan_rig")
    _text(rig, "static", "true")
    _text(rig, "pose", "6.8 6 1 0 0 0")
    publisher = ET.SubElement(
        rig,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    for tag, value in [
        ("publish_model_pose", "true"),
        ("publish_link_pose", "false"),
        ("publish_nested_model_pose", "false"),
        ("use_pose_vector_msg", "true"),
        ("static_publisher", "false"),
        ("update_frequency", 10),
    ]:
        _text(publisher, tag, value)
    link = ET.SubElement(rig, "link", name="lidar_link")
    visual = ET.SubElement(link, "visual", name="sensor_body")
    _text(visual, "pose", "0 0 -.25 0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    _text(cylinder, "radius", 0.12)
    _text(cylinder, "length", 0.15)
    sensor = ET.SubElement(link, "sensor", name="measurement_lidar", type="gpu_lidar")
    _text(sensor, "topic", "/slam_physical/lidar")
    _text(sensor, "gz_frame_id", "lidar_link")
    _text(sensor, "update_rate", 5)
    _text(sensor, "always_on", "true")
    ray = ET.SubElement(sensor, "lidar")
    scan = ET.SubElement(ray, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    for tag, value in [
        ("samples", 360),
        ("resolution", 1),
        ("min_angle", -math.pi),
        ("max_angle", math.pi),
    ]:
        _text(horizontal, tag, value)
    vertical = ET.SubElement(scan, "vertical")
    for tag, value in [
        ("samples", 32),
        ("resolution", 1),
        ("min_angle", -0.55),
        ("max_angle", 0.45),
    ]:
        _text(vertical, tag, value)
    ranges = ET.SubElement(ray, "range")
    for tag, value in [("min", 0.3), ("max", 12), ("resolution", 0.01)]:
        _text(ranges, tag, value)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    write_json(
        path.with_suffix(".json"),
        {
            "static_boxes": boxes,
            "scenario": scenario,
            "sensor": "32x360 GPU LiDAR",
            "kinematic_rig": True,
        },
    )
    return boxes
