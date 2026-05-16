import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "depth_wall_yaw.py"
SPEC = importlib.util.spec_from_file_location("depth_wall_yaw", MODULE_PATH)
depth_yaw = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = depth_yaw
SPEC.loader.exec_module(depth_yaw)


class DepthWallYawMathTest(unittest.TestCase):
    def test_rotate180_intrinsics_adjustment(self):
        intr = depth_yaw.DepthIntrinsics(width=848, height=480, fx=420.0, fy=421.0, ppx=423.5, ppy=240.25)
        adjusted = depth_yaw.adjust_intrinsics_for_rotate180(intr)
        self.assertEqual(adjusted.width, 848)
        self.assertEqual(adjusted.height, 480)
        self.assertAlmostEqual(adjusted.ppx, 423.5)
        self.assertAlmostEqual(adjusted.ppy, 238.75)

    def test_yaw_from_wall_normal(self):
        yaw = math.radians(0.53)
        normal = np.array([math.sin(yaw), 0.0, math.cos(yaw)])
        self.assertAlmostEqual(depth_yaw.yaw_from_wall_normal(normal), yaw, places=12)
        self.assertAlmostEqual(depth_yaw.yaw_from_wall_normal(-normal), yaw, places=12)

    def test_fit_plane_known_yaw(self):
        rng = np.random.default_rng(3)
        yaw = math.radians(0.75)
        normal = np.array([math.sin(yaw), 0.0, math.cos(yaw)])
        distance = 1.3
        xs = rng.uniform(-0.45, 0.45, 1000)
        ys = rng.uniform(-0.25, 0.25, 1000)
        zs = (distance - normal[0] * xs - normal[1] * ys) / normal[2]
        points = np.column_stack([xs, ys, zs])
        normal_fit, d_fit, inliers = depth_yaw.fit_plane_ransac(
            points,
            threshold_m=0.002,
            iterations=40,
            rng=np.random.default_rng(4),
        )
        self.assertGreater(np.count_nonzero(inliers), 950)
        self.assertAlmostEqual(abs(d_fit), distance, places=6)
        self.assertAlmostEqual(depth_yaw.yaw_from_wall_normal(normal_fit), yaw, places=6)

    def test_left_right_yaw_from_points(self):
        yaw = math.radians(1.0)
        normal = np.array([math.sin(yaw), 0.0, math.cos(yaw)])
        distance = 1.3
        xs = np.linspace(-0.4, 0.4, 200)
        zs = (distance - normal[0] * xs) / normal[2]
        points = np.column_stack([xs, np.zeros_like(xs), zs])
        pixels = np.column_stack([np.linspace(100.0, 700.0, 200), np.full(200, 240.0)])
        left, right, delta, lr_yaw = depth_yaw.left_right_yaw_from_points(
            points,
            pixels,
            roi_px=(100, 100, 700, 380),
            edge_band=0.2,
            min_edge_points=20,
        )
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertLess(delta, 0.0)
        self.assertAlmostEqual(lr_yaw, yaw, places=4)


if __name__ == "__main__":
    unittest.main()
