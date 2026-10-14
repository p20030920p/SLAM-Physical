"""Physics-driven differential robot in the existing measurement annex."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from slam_physical.world import _text, build_world

__all__ = ["build_drive_world", "ROBOT_START", "LIDAR_HEIGHT"]
ROBOT_START = (6.0, 6.0, 0.15, 0.0)
LIDAR_HEIGHT = 0.65


def _shape(parent, kind, values):
    shape = ET.SubElement(ET.SubElement(parent, "geometry"), kind)
    for key, value in values.items():
        _text(shape, key, value)


def _link(model, name, pose, mass, kind, values, color, friction=1.0):
    link = ET.SubElement(model, "link", name=name)
    _text(link, "pose", pose)
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", mass)
    inertia = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"):
        _text(inertia, key, 0.02 * mass)
    for key in ("ixy", "ixz", "iyz"):
        _text(inertia, key, 0)
    for mode in ("collision", "visual"):
        element = ET.SubElement(link, mode, name=mode)
        _shape(element, kind, values)
        if mode == "visual":
            material = ET.SubElement(element, "material")
            _text(material, "ambient", color)
            _text(material, "diffuse", color)
        else:
            ode = ET.SubElement(
                ET.SubElement(ET.SubElement(element, "surface"), "friction"), "ode"
            )
            _text(ode, "mu", friction)
            _text(ode, "mu2", friction)
    return link


def _lidar(link, name, topic, rows, rate):
    sensor = ET.SubElement(link, "sensor", name=name, type="gpu_lidar")
    _text(sensor, "pose", "0 0 .38 0 0 0")
    _text(sensor, "topic", topic)
    _text(sensor, "gz_frame_id", "lidar_link")
    _text(sensor, "update_rate", rate)
    _text(sensor, "always_on", "true")
    lidar = ET.SubElement(sensor, "lidar")
    scan = ET.SubElement(lidar, "scan")
    for direction, count, low, high in [
        ("horizontal", 360, -math.pi, math.pi),
        ("vertical", rows, -0.55 if rows > 1 else 0, 0.45 if rows > 1 else 0),
    ]:
        element = ET.SubElement(scan, direction)
        for key, value in [
            ("samples", count),
            ("resolution", 1),
            ("min_angle", low),
            ("max_angle", high),
        ]:
            _text(element, key, value)
    ranges = ET.SubElement(lidar, "range")
    for key, value in [("min", 0.15), ("max", 12), ("resolution", 0.01)]:
        _text(ranges, key, value)


def build_drive_world(path: Path, assets: Path, scenario="static") -> list[dict]:
    boxes = build_world(path, assets, scenario)
    root = ET.parse(path).getroot()
    world = root.find("world")
    world.set("name", "slam_drive")
    world.remove(world.find("model[@name='scan_rig']"))
    # Real wheel joints and contact dynamics; never set_pose the robot.
    robot = ET.SubElement(world, "model", name="drive_robot")
    _text(robot, "pose", "6 6 .15 0 0 0")
    chassis = _link(
        robot,
        "base_link",
        "0 0 .12 0 0 0",
        4.0,
        "box",
        {"size": ".4 .32 .14"},
        ".15 .35 .8 1",
    )
    for side, y in [("left", 0.22), ("right", -0.22)]:
        name = side + "_wheel"
        _link(
            robot,
            name,
            f"0 {y} 0 {math.pi/2} 0 0",
            0.25,
            "cylinder",
            {"radius": 0.15, "length": 0.05},
            ".1 .1 .1 1",
        )
        joint = ET.SubElement(robot, "joint", name=name + "_joint", type="revolute")
        _text(joint, "parent", "base_link")
        _text(joint, "child", name)
        axis = ET.SubElement(joint, "axis")
        ET.SubElement(axis, "xyz", expressed_in="__model__").text = "0 1 0"
    for name, x in [("rear_caster", -0.18), ("front_caster", 0.18)]:
        _link(
            robot,
            name,
            f"{x} 0 -.10 0 0 0",
            0.05,
            "sphere",
            {"radius": 0.05},
            ".2 .2 .2 1",
            0.01,
        )
        joint = ET.SubElement(robot, "joint", name=name + "_joint", type="fixed")
        _text(joint, "parent", "base_link")
        _text(joint, "child", name)
    plugin = ET.SubElement(
        robot,
        "plugin",
        filename="gz-sim-diff-drive-system",
        name="gz::sim::systems::DiffDrive",
    )
    for key, value in [
        ("left_joint", "left_wheel_joint"),
        ("right_joint", "right_wheel_joint"),
        ("wheel_separation", 0.44),
        ("wheel_radius", 0.15),
        ("topic", "/cmd_vel"),
        ("odom_topic", "/wheel_odom"),
        ("tf_topic", "/unused_wheel_tf"),
        ("frame_id", "odom"),
        ("child_frame_id", "base_footprint"),
        ("odom_publish_frequency", 30),
        ("max_linear_velocity", 0.5),
        ("max_angular_velocity", 1.0),
        ("max_linear_acceleration", 1.0),
    ]:
        _text(plugin, key, value)
    plugin = ET.SubElement(
        robot,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    for key, value in [
        ("publish_model_pose", "true"),
        ("publish_link_pose", "false"),
        ("publish_nested_model_pose", "false"),
        ("use_pose_vector_msg", "true"),
        ("static_publisher", "false"),
        ("update_frequency", 30),
    ]:
        _text(plugin, key, value)
    _lidar(chassis, "localization_lidar", "/scan", 1, 10)
    _lidar(chassis, "mapping_lidar", "/lidar3d", 32, 5)
    sensor = ET.SubElement(chassis, "sensor", name="rgbd", type="rgbd_camera")
    for key, value in [
        ("pose", ".2 0 .48 0 0 0"),
        ("topic", "/camera"),
        ("update_rate", 3),
        ("always_on", "true"),
        ("gz_frame_id", "camera_optical_frame"),
    ]:
        _text(sensor, key, value)
    camera = ET.SubElement(sensor, "camera")
    _text(camera, "horizontal_fov", 1.2)
    image = ET.SubElement(camera, "image")
    for key, value in [("width", 320), ("height", 180), ("format", "R8G8B8")]:
        _text(image, key, value)
    clip = ET.SubElement(camera, "clip")
    _text(clip, "near", 0.1)
    _text(clip, "far", 12)
    gui = ET.SubElement(world, "gui", fullscreen="false")
    # A camera aimed at the actual measurement area rather than the distant arena.
    view = ET.SubElement(gui, "plugin", filename="MinimalScene", name="3D View")
    _text(view, "engine", "ogre2")
    _text(view, "scene", "scene")
    _text(view, "camera_pose", "15 0 14 0 .9 2.4")
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return boxes
