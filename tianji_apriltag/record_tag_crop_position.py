"""Record a live AprilTag center in the exact OpenPI 224 crop coordinates."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np


def main():
    package = Path(__file__).resolve().parent
    workspace = package.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=int, required=True)
    parser.add_argument('--tag-id', type=int, default=18)
    parser.add_argument('--state-url', default='http://127.0.0.1:23846/api/state')
    parser.add_argument('--openpi-calibration', default=str(
        workspace / 'tianji_data_convert/special/duicha/shared_camera_intrinsics_5view224.json'))
    parser.add_argument('--output', default=None)
    args = parser.parse_args()

    with urlopen(args.state_url, timeout=5) as response:
        state = json.load(response)
    camera_state = next(
        (item for item in state['cameras'] if item['camera'] == args.camera), None)
    if (camera_state is None or camera_state.get('status') != 'live'
            or camera_state.get('age_s', 99) > 1):
        raise ValueError(f'camera {args.camera} does not have a fresh live frame')
    tag = next((item for item in camera_state['tags'] if item['id'] == args.tag_id), None)
    if tag is None:
        raise ValueError(f'camera {args.camera} does not currently see tag {args.tag_id}')

    calibration = json.loads(Path(args.openpi_calibration).read_text())
    camera_name = f'hik_camera_{args.camera}'
    left, top, width, height = calibration['image_preprocessing']['crop_rectangles'][camera_name]
    k = np.asarray(calibration['source_camera_matrix'], dtype=np.float64)
    d = np.asarray(
        calibration['wrist_undistortion']['coefficients'][camera_name], dtype=np.float64)
    raw_center = np.asarray(tag['center_px'], dtype=np.float64).reshape(1, 1, 2)
    rectified_center = cv2.fisheye.undistortPoints(
        raw_center, k, d, R=np.eye(3), P=k).reshape(2)
    crop_center = rectified_center - [left, top]
    inside = bool(0 <= crop_center[0] < width and 0 <= crop_center[1] < height)

    result = {
        'schema_version': 'tianji.openpi_crop_tag_position.v1',
        'recorded_at': datetime.now().astimezone().isoformat(),
        'camera': args.camera,
        'frame_id': camera_state['frame_id'],
        'ros_stamp': camera_state['stamp'],
        'tag_id': args.tag_id,
        'tag_size_m': tag['size_m'],
        'tag_center_raw_image_px': raw_center.reshape(2).tolist(),
        'tag_center_rectified_image_px': rectified_center.tolist(),
        'openpi_crop_rectangle_xywh': [left, top, width, height],
        'tag_center_in_224_crop_px': crop_center.tolist(),
        'tag_center_inside_crop': inside,
        'crop_geometric_center_px': [(width - 1) / 2, (height - 1) / 2],
        'offset_from_crop_center_px': (
            crop_center - [(width - 1) / 2, (height - 1) / 2]).tolist(),
        'tag_depth_m': tag.get('translation_m', [None, None, None])[2],
        'tag_reprojection_error_px': tag.get('reprojection_error_px'),
        'method': 'Live tag center transformed with the same fisheye model as OpenPI DirectCropper.',
    }
    output = (Path(args.output) if args.output else package.parent / 'records' /
              f'openpi_crop_tag_{args.tag_id}_position_camera_{args.camera}.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(output)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
