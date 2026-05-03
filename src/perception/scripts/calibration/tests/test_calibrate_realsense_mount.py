import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "calibrate_realsense_mount.py"
SPEC = importlib.util.spec_from_file_location("calibrate_realsense_mount", MODULE_PATH)
calib = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = calib
SPEC.loader.exec_module(calib)


class CalibrationMathTest(unittest.TestCase):
    def test_rotate180_intrinsics_adjustment(self):
        camera_matrix = np.array(
            [
                [100.0, 0.0, 30.0],
                [0.0, 101.0, 40.0],
                [0.0, 0.0, 1.0],
            ]
        )
        dist = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        adjusted_k, adjusted_d = calib.adjust_intrinsics_for_rotate180(camera_matrix, dist, 640, 480)
        self.assertAlmostEqual(adjusted_k[0, 2], 609.0)
        self.assertAlmostEqual(adjusted_k[1, 2], 439.0)
        np.testing.assert_allclose(adjusted_d, [1.0, 2.0, -3.0, -4.0, 5.0])

    def test_charuco_corner_id_mapping(self):
        spec = calib.normalize_target_spec(dict(calib.DEFAULT_SPEC))
        first = calib.charuco_id_to_target_point(0, spec)
        last = calib.charuco_id_to_target_point(34, spec)
        np.testing.assert_allclose(first, [0.06, 0.09, 0.0], atol=1e-7)
        np.testing.assert_allclose(last, [-0.06, -0.09, 0.0], atol=1e-7)

    def test_repo_euler_round_trip(self):
        expected = (0.11, -0.07, 0.025)
        rotation = calib.compose_repo_rotation(*expected)
        actual = calib.extract_repo_euler(rotation)
        for got, want in zip(actual, expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_cpp_constant_expression_parser(self):
        path = Path(__file__).with_name("_tmp_constants.h")
        try:
            path.write_text(
                "static constexpr std::array<double, 6> REALSENSE_TF_REAL = "
                "{-0.09, -0.032, 0.260, -0.009774, 1.85*M_PI/180.0, 0.011};\n",
                encoding="utf-8",
            )
            values = calib.parse_numeric_array_from_constants(path, "REALSENSE_TF_REAL")
            self.assertIsNotNone(values)
            self.assertAlmostEqual(values[4], 1.85 * math.pi / 180.0)
        finally:
            if path.exists():
                path.unlink()

    def test_runtime_patch_intrinsics_use_runtime_size(self):
        params = calib.runtime_camera_params_from_constants(
            [607.4, 607.0, 322.9, 244.4],
            "rotate180",
            640,
            480,
        )
        self.assertAlmostEqual(params[0], 607.4)
        self.assertAlmostEqual(params[1], 607.0)
        self.assertAlmostEqual(params[2], 316.1)
        self.assertAlmostEqual(params[3], 234.6)


if __name__ == "__main__":
    unittest.main()
