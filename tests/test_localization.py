import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from slam_physical.drive_world import build_drive_world
from slam_physical.dynamic_scene import actor_boxes
from slam_physical.localization_benchmark_report import METHODS, aggregate
from slam_physical.localization_compare import compare
from slam_physical.localization_maps import build_maps, rgbd_geometry
from slam_physical.localization_report import localization_report
from slam_physical.occupancy import project_cloud, reference_grid, save_grid
from slam_physical.pose_tools import interpolate_pose
from slam_physical.provided_mapping_poses import interpolate_transforms


class LocalizationTests(unittest.TestCase):
    def test_unsupported_training_pose_is_excluded_from_lidar_and_rgbd_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            source = folder / "source"
            results = source / "rgbd/room0/results"
            results.mkdir(parents=True)
            poses = np.repeat(np.eye(4)[None], 3, axis=0)
            archive = {f"scan_{i:04d}": np.array([[2.0, 2.0, 0.65]]) for i in range(3)}
            np.savez(
                source / "mapping_scans.npz",
                **archive,
                poses=poses,
                stamps=np.arange(3),
            )
            np.savetxt(source / "rgbd/room0/traj.txt", poses.reshape(-1, 16))
            (source / "rgbd/cam_params.json").write_text(
                json.dumps(
                    {"camera": {"fx": 1, "fy": 1, "cx": 0, "cy": 0, "scale": 1000}}
                )
            )
            for i in range(3):
                Image.fromarray(np.full((2, 2), 1000, dtype=np.uint16)).save(
                    results / f"depth{i:06d}.png"
                )
                Image.fromarray(np.zeros((2, 2, 3), dtype=np.uint8)).save(
                    results / f"frame{i:06d}.jpg"
                )
            poses[0] = np.nan
            np.save(source / "provided_lidar_poses.npy", poses)
            np.save(source / "provided_rgbd_poses.npy", poses)
            (source / "provided_pose_record.json").write_text("{}")
            points, _, ids = rgbd_geometry(source, 3, provided=True)
            self.assertEqual(ids, [1, 2])
            self.assertTrue(np.isfinite(points).all())
            manifest = build_maps(
                source,
                folder / "maps",
                folder / "runtime",
                methods=("raw",),
                pose_source="provided",
            )
            self.assertEqual(manifest["lidar_observation_ids"], [1, 2])
            self.assertEqual(manifest["semantic_available_original_ids"], [1, 2])

    def test_provided_training_pose_interpolation_preserves_rotation_and_rejects_extrapolation(
        self,
    ):
        stamps = np.array([0.0, 1.0])
        positions = np.array([[0.0, 0.0, 0.15], [2.0, 0.0, 0.15]])
        quaternions = np.array(
            [[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)]]
        )
        pose = interpolate_transforms(stamps, positions, quaternions, [0.5])[0]
        self.assertAlmostEqual(pose[0, 3], 1.0)
        self.assertAlmostEqual(np.arctan2(pose[1, 0], pose[0, 0]), np.pi / 4)
        with self.assertRaisesRegex(ValueError, "extrapolate"):
            interpolate_transforms(stamps, positions, quaternions, [1.01])

    def test_dynamic_obstacles_are_excluded_from_the_clean_reference_map(self):
        import xml.etree.ElementTree as ET

        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            asset = (
                folder / "assets/src/race_gazebo/models/competition_map_dae/model.sdf"
            )
            asset.parent.mkdir(parents=True)
            asset.write_text("<sdf/>")
            for scene, expected in [("static", 0), ("dynamic", 3)]:
                path = folder / (scene + ".sdf")
                boxes = build_drive_world(path, folder / "assets", scene)
                root = ET.parse(path).getroot()
                models = [item.get("name") for item in root.findall("world/model")]
                self.assertEqual(
                    sum(
                        name in models
                        for name in ["west_cart", "south_cart", "north_cart"]
                    ),
                    expected,
                )
                self.assertNotIn("scan_rig", models)
                self.assertNotIn("west_cart", [box["name"] for box in boxes])
                self.assertFalse(
                    json.loads(path.with_suffix(".json").read_text())["kinematic_rig"]
                )

    def test_automated_robot_corridor_has_clearance_from_moving_boxes(self):
        for t in np.linspace(0, 80, 500):
            robot = np.array(
                [
                    6 + 1.35 * np.sin(t / 1.35 * 0.25),
                    7.35 - 1.35 * np.cos(t / 1.35 * 0.25),
                ]
            )
            for actor in actor_boxes(t):
                separation = np.maximum(
                    np.abs(robot - actor["center"][:2])
                    - np.array(actor["size"][:2]) / 2,
                    0,
                )
                self.assertGreater(np.linalg.norm(separation), 0.3)

    def test_map_training_recording_cannot_be_used_for_testing(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "maps.json").write_text(
                json.dumps({"mapping_recording": str(folder)})
            )
            with self.assertRaisesRegex(ValueError, "held-out"):
                compare(folder, folder, folder / "comparison")

    def test_aggregation_compares_each_method_to_its_own_sensor_baseline(self):
        rows = []
        keys = [
            "position_rmse_m",
            "post_warmup_5s_position_rmse_m",
            "position_p95_m",
            "position_max_m",
            "yaw_rmse_deg",
            "observed_scan_to_pose_wall_p95_ms",
        ]
        for method in METHODS:
            family = "lidar" if method in METHODS[:3] else "rgbd_geometry_projection"
            for value in [1.0, 2.0, 3.0]:
                rows.append(
                    {
                        "method": method,
                        "family": family,
                        "status": "scored",
                        **{key: value for key in keys},
                    }
                )
        result = aggregate(rows, "dynamic")
        self.assertEqual(result[4]["baseline"], "rgbd-raw")
        self.assertEqual(result[1]["baseline"], "raw")
        self.assertEqual(result[1]["post_warmup_5s_position_rmse_m_std"], 1.0)
        rows[0]["status"] = "failed"
        with self.assertRaisesRegex(ValueError, "failed"):
            aggregate(rows, "dynamic")

    def test_warmup_metric_does_not_hide_initialization_in_full_rmse(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            with (folder / "amcl_samples.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    [
                        "stamp_sim_s",
                        "xy_error_m",
                        "yaw_error_rad",
                        "scan_to_pose_observed_wall_ms",
                        "pose_gap_wall_s",
                    ]
                )
                writer.writerows(
                    [[0, 1, 0, 10, ""], [6, 0.1, 0, 10, 6], [7, 0.1, 0, 10, 1]]
                )
            (folder / "live_error.csv").write_text("stamp_sim_s\n")
            summary = localization_report(folder)
            self.assertAlmostEqual(summary["post_warmup_5s_position_rmse_m"], 0.1)
            self.assertGreater(summary["position_rmse_m"], 0.5)

    def test_timestamp_matching_rejects_unsynchronized_truth(self):
        self.assertIsNone(interpolate_pose([(1.0, 0.0, 0.0, 0.0)], 2.0))
        self.assertIsNone(
            interpolate_pose([(1.0, 0.0, 0.0, 0.0), (2.0, 1.0, 0.0, 0.0)], 1.5)
        )

    def test_interpolation_crosses_pi_without_false_full_rotation(self):
        pose, _ = interpolate_pose(
            [(1.0, 0.0, 0.0, np.deg2rad(179)), (1.1, 1.0, 0.0, np.deg2rad(-179))], 1.05
        )
        self.assertAlmostEqual(pose[0], 0.5)
        self.assertAlmostEqual(abs(pose[2]), np.pi)

    def test_floor_is_not_an_amcl_wall(self):
        grid = reference_grid([{"center": [6, 6, -0.1], "size": [12, 12, 0.2]}])
        self.assertEqual(grid[140, 140], 254)

    def test_projection_preserves_unknown_and_deleted_occupied_cells(self):
        cloud = np.array([[2.0, 2.0, 0.65]])
        origins = np.array([[1.0, 2.0, 0.65]])
        raw = project_cloud(cloud, cloud, origins)
        removed = project_cloud(np.empty((0, 3)), cloud, origins)
        self.assertEqual(raw[60, 60], 0)
        self.assertEqual(removed[60, 60], 254)
        self.assertEqual(raw[200, 200], 205)
        with tempfile.TemporaryDirectory() as directory:
            path = save_grid(Path(directory) / "map", raw, {"method": "test"})
            self.assertTrue(path.is_file())


if __name__ == "__main__":
    unittest.main()
