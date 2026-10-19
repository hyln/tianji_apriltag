import unittest
from types import SimpleNamespace
import cv2
import numpy as np
from tianji_apriltag.detection import Detector, estimate_pose

class DetectionTests(unittest.TestCase):
    def test_pdf(self):
        image = cv2.imread('/tmp/apriltag_pdf.png')
        if image is None:
            self.skipTest('Render reference PDF to /tmp/apriltag_pdf.png first')
        tags = Detector().detect(image)
        self.assertEqual({t['id']: t['size_m'] for t in tags}, {18: .034, 19: .024})

    def test_pose_and_invalid_calibration(self):
        k = np.array([[830., 0, 720], [0, 830, 540], [0, 0, 1]])
        info = SimpleNamespace(k=k.flatten(), d=[0]*5, width=1440, height=1080,
            distortion_model='plumb_bob', binning_x=0, binning_y=0,
            roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0))
        s = .042/2
        obj = np.array([[-s,s,0], [s,s,0], [s,-s,0], [-s,-s,0]])
        r = np.array([2.9, .1, .05]); t = np.array([.02, -.01, .4])
        for model, d in [('plumb_bob', [0]*5), ('equidistant', [-.08,.005,-.004,.002])]:
            info.distortion_model, info.d = model, d
            if model == 'equidistant':
                corners, _ = cv2.fisheye.projectPoints(obj.reshape(-1,1,3), r, t, k, np.array(d))
            else:
                corners, _ = cv2.projectPoints(obj, r, t, k, np.array(d, dtype=float))
            pose = estimate_pose(corners, .042, info, (1080,1440,3))
            np.testing.assert_allclose(pose['translation_m'], t, atol=1e-5)
            self.assertLess(pose['reprojection_error_px'], 1e-4)
        info.d = [0]*5
        with self.assertRaises(ValueError):
            estimate_pose(corners, .042, info, (1080,1440,3))
        info.distortion_model = 'plumb_bob'
        info.k = [830,0,720,0,0,830,540,0,0]
        with self.assertRaises(ValueError):
            estimate_pose(corners, .042, info, (1080,1440,3))

if __name__ == '__main__':
    unittest.main()
