import unittest
from tianji_apriltag.detection import center_from_corners, center_midpoint, center_separation, REFERENCE_CENTER_DISTANCE_M

class CenterDistanceTests(unittest.TestCase):
    def test_perspective_center_is_diagonal_intersection(self):
        self.assertEqual(center_from_corners([[0, 0], [4, 0], [3, 2], [1, 2]]),
                         [2.0, 4.0 / 3.0])

    def test_two_poses_from_one_frame(self):
        tags = [{'id': 18, 'translation_m': [0, 0, .1]},
                {'id': 19, 'translation_m': [0, .033, .144]}]
        self.assertAlmostEqual(center_separation(tags), .055)
        self.assertEqual(center_midpoint(tags), [0.0, .0165, .122])
        self.assertEqual(REFERENCE_CENTER_DISTANCE_M, .055)
        self.assertIsNone(center_separation(tags[:1]))
        self.assertIsNone(center_midpoint(tags[:1]))
        self.assertIsNone(center_separation([tags[0], {'id': 19, 'pose_error': 'bad calibration'}]))

if __name__ == '__main__':
    unittest.main()
