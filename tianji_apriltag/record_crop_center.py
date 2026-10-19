"""Record where an OpenPI crop center ray intersects the live AprilTag plane."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np


def unit(vector):
    vector = np.asarray(vector, dtype=np.float64)
    length = np.linalg.norm(vector)
    if length <= 1e-12:
        raise ValueError('Cannot normalize a zero vector')
    return vector / length


def bilinear(array, x, y):
    x0, y0 = int(np.floor(x)), int(np.floor(y))
    ax, ay = x - x0, y - y0
    return float((1 - ax) * (1 - ay) * array[y0, x0]
                 + ax * (1 - ay) * array[y0, x0 + 1]
                 + (1 - ax) * ay * array[y0 + 1, x0]
                 + ax * ay * array[y0 + 1, x0 + 1])


def record(state, camera, openpi, pose_calibration):
    result = next((item for item in state['cameras'] if item['camera'] == camera), None)
    if result is None or result.get('status') != 'live' or result.get('age_s', 99) > 1:
        raise ValueError(f'camera {camera} does not have a fresh live frame')
    tags = {tag['id']: tag for tag in result['tags']}
    if not all(tag_id in tags and 'translation_m' in tags[tag_id]
               and 'rotation_vector_rad' in tags[tag_id] for tag_id in (18, 19)):
        raise ValueError(f'camera {camera} must see valid ID 18 and 19 in the same frame')

    camera_name = f'hik_camera_{camera}'
    left, top, width, height = openpi['image_preprocessing']['crop_rectangles'][camera_name]
    crop_center = np.array([(width - 1) / 2, (height - 1) / 2], dtype=np.float64)
    rectified_center = crop_center + [left, top]

    openpi_k = np.asarray(openpi['source_camera_matrix'], dtype=np.float64)
    openpi_d = np.asarray(openpi['wrist_undistortion']['coefficients'][camera_name],
                          dtype=np.float64)
    source_size = (int(openpi['source_image_width']), int(openpi['source_image_height']))
    map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
        openpi_k, openpi_d, np.eye(3), openpi_k, source_size, cv2.CV_32FC1)
    raw_center = np.array([
        bilinear(map_x, *rectified_center),
        bilinear(map_y, *rectified_center),
    ])

    pinned = pose_calibration['cameras'][str(camera)]
    pose_k = np.asarray(pinned['camera_matrix'], dtype=np.float64)
    pose_d = np.asarray(pinned['distortion_coefficients'], dtype=np.float64)
    normalized = cv2.fisheye.undistortPoints(
        raw_center.reshape(1, 1, 2), pose_k, pose_d).reshape(2)
    ray = unit([normalized[0], normalized[1], 1.0])

    p18 = np.asarray(tags[18]['translation_m'], dtype=np.float64)
    p19 = np.asarray(tags[19]['translation_m'], dtype=np.float64)
    pair_midpoint = (p18 + p19) / 2
    normals = []
    for tag_id in (18, 19):
        rotation, _ = cv2.Rodrigues(
            np.asarray(tags[tag_id]['rotation_vector_rad'], dtype=np.float64))
        normal = unit(rotation[:, 2])
        if normals and np.dot(normal, normals[0]) < 0:
            normal = -normal
        normals.append(normal)
    plane_normal = unit(normals[0] + normals[1])
    denominator = float(np.dot(plane_normal, ray))
    if abs(denominator) < 1e-6:
        raise ValueError('crop center ray is parallel to the AprilTag plane')
    distance_along_ray = float(np.dot(plane_normal, pair_midpoint) / denominator)
    if distance_along_ray <= 0:
        raise ValueError('crop center plane intersection is behind the camera')
    intersection = ray * distance_along_ray

    vertical = unit(p19 - p18)
    horizontal = unit(np.cross(vertical, plane_normal))
    if horizontal[0] < 0:
        horizontal = -horizontal
    normal = unit(np.cross(horizontal, vertical))
    offset = intersection - pair_midpoint
    board_offset = np.array([
        np.dot(offset, horizontal),
        np.dot(offset, vertical),
        np.dot(offset, normal),
    ])

    return {
        'schema_version': 'tianji.openpi_crop_tag_reference.v1',
        'recorded_at': datetime.now().astimezone().isoformat(),
        'camera': camera,
        'frame_id': result['frame_id'],
        'ros_stamp': result['stamp'],
        'reference_tags': {
            'ids': [18, 19],
            'sizes_m': {'18': tags[18]['size_m'], '19': tags[19]['size_m']},
            'measured_center_distance_m': result['tag_center_distance_m'],
            'nominal_center_distance_m': result['reference_center_distance_m'],
            'pair_midpoint_camera_m': pair_midpoint.tolist(),
            'reprojection_error_px': {
                '18': tags[18]['reprojection_error_px'],
                '19': tags[19]['reprojection_error_px'],
            },
        },
        'openpi_crop': {
            'rectangle_xywh_in_rectified_image': [left, top, width, height],
            'center_in_crop_px': crop_center.tolist(),
            'center_in_rectified_image_px': rectified_center.tolist(),
            'center_corresponding_raw_image_px': raw_center.tolist(),
        },
        'view_center_on_tag_plane': {
            'camera_xyz_m': intersection.tolist(),
            'camera_depth_z_m': float(intersection[2]),
            'distance_from_camera_m': float(np.linalg.norm(intersection)),
            'offset_from_tag_pair_midpoint_camera_xyz_m': offset.tolist(),
            'offset_from_tag_pair_midpoint_board_xy_normal_m': board_offset.tolist(),
            'board_axes': 'x=paper horizontal, y=ID18 toward ID19, normal=paper normal',
        },
        'method': ('OpenPI rectified crop center -> corresponding raw pixel -> camera ray; '
                   'ray intersected with the plane averaged from live ID 18/19 poses.'),
    }


def main():
    package = Path(__file__).resolve().parent
    workspace = package.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--state-url', default='http://127.0.0.1:23846/api/state')
    parser.add_argument('--camera', type=int, default=7)
    parser.add_argument('--openpi-calibration', default=str(
        workspace / 'tianji_data_convert/special/duicha/shared_camera_intrinsics_5view224.json'))
    parser.add_argument('--pose-calibration', default=str(package / 'calibration.json'))
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    with urlopen(args.state_url, timeout=5) as response:
        state = json.load(response)
    openpi = json.loads(Path(args.openpi_calibration).read_text())
    pose_calibration = json.loads(Path(args.pose_calibration).read_text())
    value = record(state, args.camera, openpi, pose_calibration)
    output = (Path(args.output) if args.output else package.parent / 'records' /
              f'openpi_crop_view_center_camera_{args.camera}_tag18_19.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    print(output)
    print(json.dumps(value['view_center_on_tag_plane'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
