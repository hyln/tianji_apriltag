import cv2
import numpy as np

SIZES = {18: 0.034, 19: 0.024}
REFERENCE_CENTER_DISTANCE_M = 0.055


def center_from_corners(corners):
    """Image center from the intersection of a tag's two diagonals."""
    points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    directions = np.column_stack((points[2] - points[0], points[1] - points[3]))
    try:
        t, _ = np.linalg.solve(directions, points[1] - points[0])
    except np.linalg.LinAlgError:
        return points.mean(axis=0).tolist()
    return (points[0] + t * (points[2] - points[0])).tolist()


def pair_centers(tags):
    """Return the two valid 3D tag centers from a single camera frame."""
    by_id = {tag['id']: tag for tag in tags if tag['id'] in SIZES and 'translation_m' in tag}
    if set(by_id) != set(SIZES):
        return None
    p18 = np.asarray(by_id[18]['translation_m'], dtype=np.float64)
    p19 = np.asarray(by_id[19]['translation_m'], dtype=np.float64)
    if p18.shape != (3,) or p19.shape != (3,) or not np.isfinite(p18).all() or not np.isfinite(p19).all():
        return None
    return p18, p19


def center_separation(tags):
    """Euclidean 18-to-19 center distance in a single camera frame, or None."""
    pair = pair_centers(tags)
    return None if pair is None else float(np.linalg.norm(pair[1] - pair[0]))


def center_midpoint(tags):
    """3D midpoint of the tag-center segment in camera optical coordinates."""
    pair = pair_centers(tags)
    return None if pair is None else ((pair[0] + pair[1]) / 2).tolist()

class Detector:
    def __init__(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters')
                  else cv2.aruco.DetectorParameters_create())
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.dictionary, self.params = dictionary, params
        self.detector = cv2.aruco.ArucoDetector(dictionary, params) if hasattr(cv2.aruco, 'ArucoDetector') else None

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, _ = (self.detector.detectMarkers(gray) if self.detector else
                           cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params))
        return [] if ids is None else [dict(id=int(i), size_m=SIZES[int(i)],
                    corners_px=c.reshape(4, 2).tolist()) for c, i in zip(corners, ids.flatten()) if int(i) in SIZES]


def estimate_pose(corners, size, info, shape):
    """Tag axes: x right, y up, z out; OpenCV canonical decoded corner order."""
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    if (info.width != shape[1] or info.height != shape[0] or
        not np.isfinite(k).all() or not np.isfinite(d).all() or
        k[0, 0] <= 0 or k[1, 1] <= 0 or
        not np.allclose(k[2], [0, 0, 1]) or abs(k[1, 0]) > 1e-6 or abs(k[0, 1]) > 1e-6):
        raise ValueError('Invalid calibration or image/CameraInfo resolution mismatch')
    if any(info.roi.__getattribute__(a) for a in ('x_offset', 'y_offset', 'width', 'height')) or info.binning_x > 1 or info.binning_y > 1:
        raise ValueError('ROI/binning calibration requires explicit adjustment')
    pixels = np.asarray(corners, dtype=np.float64).reshape(4, 1, 2)
    if info.distortion_model == 'equidistant':
        if d.size != 4:
            raise ValueError('equidistant requires exactly 4 fisheye coefficients; confirm calibration model')
        normalized = cv2.fisheye.undistortPoints(pixels, k, d)
    elif info.distortion_model in ('plumb_bob', 'rational_polynomial'):
        if d.size not in (4, 5, 8, 12, 14):
            raise ValueError('Unsupported pinhole distortion coefficients')
        normalized = cv2.undistortPoints(pixels, k, d)
    else:
        raise ValueError('Unsupported distortion model: ' + info.distortion_model)
    s = size / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, normalized, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for r, t in zip(rvecs, tvecs):
        rotation = cv2.Rodrigues(r)[0]
        if np.min((rotation @ obj.T + t)[2]) <= 0:
            continue
        if info.distortion_model == 'equidistant':
            projected, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), r, t, k, d)
        else:
            projected, _ = cv2.projectPoints(obj, r, t, k, d)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-pixels.reshape(4, 2))**2, axis=1))))
        candidates.append((error, r, t))
    if not ok or not candidates:
        raise ValueError('No positive-depth pose')
    error, r, t = min(candidates, key=lambda x: x[0])
    return dict(translation_m=t.flatten().tolist(), rotation_vector_rad=r.flatten().tolist(),
                reprojection_error_px=error, distance_m=float(np.linalg.norm(t)),
                pose_candidates=len(candidates))
