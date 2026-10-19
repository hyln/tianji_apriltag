"""Pinned raw-image fisheye calibration for the two wrist cameras."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np


def load_calibration():
    config = json.loads(Path(__file__).with_name('calibration.json').read_text())
    if config['distortion_model'] != 'equidistant':
        raise ValueError('Expected equidistant wrist calibration')
    width, height = config['image_width'], config['image_height']
    cameras = {}
    for key, entry in config['cameras'].items():
        k = np.asarray(entry['camera_matrix'], dtype=np.float64)
        d = np.asarray(entry['distortion_coefficients'], dtype=np.float64)
        if (k.shape != (3, 3) or d.shape != (4,) or not np.isfinite(k).all() or
                not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or
                not np.allclose(k[2], [0, 0, 1])):
            raise ValueError(f'Invalid pinned calibration for camera {key}')
        cameras[int(key)] = SimpleNamespace(
            k=k.flatten().tolist(), d=d.tolist(), width=width, height=height,
            distortion_model='equidistant', binning_x=0, binning_y=0,
            roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0))
    return cameras
