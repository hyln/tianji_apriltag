import unittest
import cv2
import numpy as np
from tianji_apriltag.calibration import load_calibration
from tianji_apriltag.detection import estimate_pose

class PinnedCalibrationTests(unittest.TestCase):
    def test_each_camera_recovers_fisheye_projection(self):
        calibrations = load_calibration()
        self.assertEqual(set(calibrations), {6, 7})
        half = .034 / 2
        object_points = np.array([[-half, half, 0], [half, half, 0],
                                  [half, -half, 0], [-half, -half, 0]], dtype=float)
        rotation = np.array([2.9, .1, .05])
        translation = np.array([.02, -.01, .4])
        for camera, info in calibrations.items():
            with self.subTest(camera=camera):
                self.assertEqual((info.width, info.height), (1200, 1000))
                k = np.asarray(info.k).reshape(3, 3)
                pixels, _ = cv2.fisheye.projectPoints(object_points.reshape(-1, 1, 3),
                    rotation, translation, k, np.asarray(info.d))
                result = estimate_pose(pixels, .034, info, (1000, 1200, 3))
                np.testing.assert_allclose(result['translation_m'], translation, atol=1e-5)
                self.assertLess(result['reprojection_error_px'], 1e-4)
                with self.assertRaises(ValueError):
                    estimate_pose(pixels, .034, info, (1080, 1440, 3))

if __name__ == '__main__':
    unittest.main()
