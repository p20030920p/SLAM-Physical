"""Declared moving clutter: simulator obstacles, never localization inputs."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

ACTORS = (
    {"name": "west_cart", "size": [0.65, 1.4, 1.5]},
    {"name": "south_cart", "size": [1.4, 0.65, 1.5]},
    {"name": "north_cart", "size": [1.5, 0.65, 1.5]},
)


def actor_boxes(simulation_time: float, phase: float = 0.0) -> list[dict]:
    t = simulation_time + phase
    centers = (
        [3.9, 7.2 + 1.6 * math.sin(2 * math.pi * t / 16), 0.75],
        [6.2 + 2.0 * math.sin(2 * math.pi * t / 20 + 0.7), 4.8, 0.75],
        [6.2 + 2.0 * math.sin(2 * math.pi * t / 18 + 1.5), 10.1, 0.75],
    )
    return [{**actor, "center": center} for actor, center in zip(ACTORS, centers)]


def drive_command(elapsed: float, route: str) -> tuple[float, float]:
    """Fixed commands in simulation time; feedback truth is never used to drive."""
    if route == "legacy":
        return (0.2 if elapsed < 8 else 0.0, 0.3 if 8 <= elapsed < 12 else 0.0)
    # Two different speeds on the same safe circular corridor. The test is a
    # separately acquired drive, not reused mapping observations.
    speed = 0.25 if route == "mapping" else 0.21
    return (speed, speed / 1.35) if elapsed >= 2 else (0.0, 0.0)


def control(run: Path, phase: float):
    import rclpy
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from rclpy.parameter import Parameter

    rclpy.init()
    observer = rclpy.create_node(
        "dynamic_scene_controller",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    transport = Node()
    last_sim = -1.0
    failures = 0
    started = time.monotonic()
    try:
        with (run / "actor_commands.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_sim_s", "name", "x", "y", "z", "service_accepted"])
            while rclpy.ok() and not (run / "STOP").exists():
                rclpy.spin_once(observer, timeout_sec=0.02)
                sim = observer.get_clock().now().nanoseconds * 1e-9
                if sim <= 0 or sim - last_sim < 0.1:
                    continue
                request = Pose_V()
                boxes = actor_boxes(sim, phase)
                for box in boxes:
                    pose = request.pose.add()
                    pose.name = box["name"]
                    pose.position.x, pose.position.y, pose.position.z = box["center"]
                    pose.orientation.w = 1.0
                result, response = transport.request(
                    "/world/slam_drive/set_pose_vector", request, Pose_V, Boolean, 500
                )
                accepted = bool(result and response.data)
                failures = 0 if accepted else failures + 1
                for box in boxes:
                    writer.writerow([sim, box["name"], *box["center"], accepted])
                stream.flush()
                last_sim = sim
                if failures >= 20 and time.monotonic() - started > 15:
                    raise RuntimeError(
                        "Gazebo rejected moving obstacle poses repeatedly"
                    )
    except KeyboardInterrupt:
        pass
    finally:
        observer.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase", type=float, default=0.0)
    args = parser.parse_args()
    control(args.output, args.phase)


if __name__ == "__main__":
    main()
