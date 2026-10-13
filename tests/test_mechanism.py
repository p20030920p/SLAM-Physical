import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from slam_physical.execution import run_logged
from slam_physical.experiment import measure
from slam_physical.geometry import (
    revised_poses,
    support_mask,
    surface_distance,
    world_points,
)
from slam_physical.storage import read_pcd, write_pcd


class MechanismTests(unittest.TestCase):
    def test_revision_changes_only_declared_history(self):
        truth = np.repeat(np.eye(4)[None], 40, axis=0)
        old = revised_poses(truth, 0.3)
        changed = np.flatnonzero(np.max(np.abs(old - truth), axis=(1, 2)) > 0)
        np.testing.assert_array_equal(changed, np.arange(9, 16))
        self.assertAlmostEqual(old[15, 0, 3], 0.3)
        np.testing.assert_array_equal(revised_poses(truth, 0), truth)

    def test_frame_support_is_not_point_count(self):
        points = np.array([[0.01, 0.01, 0.01]] * 4)
        self.assertFalse(support_mask(points, np.array([0, 0, 0, 1]), 0.2, 3).any())
        self.assertTrue(support_mask(points, np.array([0, 1, 2, 2]), 0.2, 3).all())

    def test_source_order_survives_geometry_revision(self):
        scans = [np.array([[1.0, 0, 0], [2.0, 0, 0]]), np.array([[3.0, 0, 0]])]
        poses = np.repeat(np.eye(4)[None], 2, axis=0)
        poses[1, 0, 3] = 5
        points, ids = world_points(scans, poses)
        np.testing.assert_array_equal(points[:, 0], [1, 2, 8])
        np.testing.assert_array_equal(ids, [0, 0, 1])

    def test_distance_is_to_surface_not_inside_volume(self):
        boxes = [{"center": [0, 0, 0], "size": [2, 2, 2]}]
        distance = surface_distance(np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]]), boxes)
        np.testing.assert_allclose(distance, [1, 0, 1])

    def test_pcd_has_no_label_or_source_id_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "cloud.pcd"
            xyz = np.array([[1.25, 2.0, 3.0], [4.0, 5.0, 6.0]])
            write_pcd(p, xyz, origin=[10, 20, 30])
            self.assertIn(b"FIELDS x y z\n", p.read_bytes())
            np.testing.assert_array_equal(read_pcd(p), xyz)

    def test_unknown_labels_do_not_inflate_static_or_dynamic_accuracy(self):
        points = np.array([[1.0, 0, 0], [1.0, 0, 0], [1.0, 0, 0]])
        boxes = [{"center": [0, 0, 0], "size": [2, 2, 2]}]
        result = measure(
            points,
            np.array([True, False, True]),
            points[:1],
            boxes,
            np.array([0, 1, -1]),
        )
        self.assertEqual(result["static_retention_percent"], 100)
        self.assertEqual(result["dynamic_accuracy"], 100)
        self.assertEqual(result["HA"], 100)
        self.assertEqual(result["unclassified_points"], 1)
        static = measure(
            points, np.ones(3, dtype=bool), points[:1], boxes, np.zeros(3, dtype=int)
        )
        self.assertIsNone(static["dynamic_accuracy"])
        self.assertIsNone(static["HA"])

    @unittest.skipUnless(sys.platform == "linux", "POSIX process-group cancellation")
    def test_stop_interrupts_silent_native_process(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            timer = threading.Timer(0.3, lambda: (path / "STOP").touch())
            timer.start()
            started = time.monotonic()
            try:
                with self.assertRaises(InterruptedError):
                    run_logged(
                        [sys.executable, "-c", "import time; time.sleep(30)"],
                        path,
                        path / "native.log",
                        path,
                    )
            finally:
                timer.cancel()
                timer.join()
            self.assertLess(time.monotonic() - started, 5)


if __name__ == "__main__":
    unittest.main()
