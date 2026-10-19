"""OpenCV viewer for the exact 224x224 crop views used by OpenPI."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from tianji_data_convert.common.wrist_undistortion import DirectCropper


class CropView(Node):
    def __init__(self, calibration_path, scale):
        super().__init__('tianji_openpi_crop_view')
        calibration = json.loads(Path(calibration_path).read_text())
        self.processors = {
            camera: DirectCropper(calibration, camera, 224)
            for camera in ('hik_camera_6', 'hik_camera_7')
        }
        self.bridge = CvBridge()
        self.frames = {}
        self.scale = scale
        self.window = 'OpenPI wrist crops (Q / ESC to close)'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 448 * scale, 224 * scale)
        for index in (6, 7):
            camera = f'hik_camera_{index}'
            topic = f'/perception/sensors/{camera}/image'
            self.create_subscription(
                Image, topic, lambda msg, name=camera: self.receive(name, msg),
                qos_profile_sensor_data)
        rectangles = calibration['image_preprocessing']['crop_rectangles']
        self.get_logger().info(
            f"OpenPI crop view: cam6={rectangles['hik_camera_6']}, "
            f"cam7={rectangles['hik_camera_7']}")

    def receive(self, camera, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frames[camera] = self.processors[camera](bgr)
        except Exception as exc:
            self.get_logger().error(f'{camera}: {exc}')

    def show(self):
        panels = []
        for camera, title in (('hik_camera_6', 'CAM 6 / LEFT'),
                              ('hik_camera_7', 'CAM 7 / RIGHT')):
            frame = self.frames.get(camera)
            if frame is None:
                frame = np.zeros((224, 224, 3), dtype=np.uint8)
                cv2.putText(frame, 'WAITING', (58, 118),
                            cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 200, 255), 2)
            else:
                frame = frame.copy()
            cv2.rectangle(frame, (0, 0), (223, 25), (0, 0, 0), -1)
            cv2.putText(frame, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (255, 255, 255), 1, cv2.LINE_AA)
            # A 224x224 crop has its geometric center between pixels; mark the
            # nearest display pixel so the recorded view ray is visible.
            cv2.drawMarker(frame, (112, 112), (0, 0, 0),
                           cv2.MARKER_CROSS, 19, 4)
            cv2.drawMarker(frame, (112, 112), (0, 255, 255),
                           cv2.MARKER_CROSS, 15, 2)
            cv2.putText(frame, 'CENTER', (118, 106), cv2.FONT_HERSHEY_SIMPLEX,
                        .35, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, 'CENTER', (118, 106), cv2.FONT_HERSHEY_SIMPLEX,
                        .35, (0, 255, 255), 1, cv2.LINE_AA)
            panels.append(frame)
        canvas = cv2.hconcat(panels)
        if self.scale != 1:
            canvas = cv2.resize(canvas, None, fx=self.scale, fy=self.scale,
                                interpolation=cv2.INTER_NEAREST)
        cv2.imshow(self.window, canvas)
        return cv2.waitKey(1) & 0xFF


def main():
    default_calibration = (Path(__file__).resolve().parents[2] /
                           'tianji_data_convert/special/duicha/'
                           'shared_camera_intrinsics_5view224.json')
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', default=str(default_calibration))
    parser.add_argument('--scale', type=int, default=3)
    args, ros_args = parser.parse_known_args()
    if args.scale < 1:
        parser.error('--scale must be at least 1')
    rclpy.init(args=ros_args)
    node = CropView(args.calibration, args.scale)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            if node.show() in (ord('q'), ord('Q'), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
