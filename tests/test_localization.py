import tempfile
import unittest
from pathlib import Path

import numpy as np

from slam_physical.occupancy import project_cloud, reference_grid, save_grid
from slam_physical.pose_tools import interpolate_pose


class LocalizationTests(unittest.TestCase):
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
