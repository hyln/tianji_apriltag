import argparse
import json
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .detection import Detector, estimate_pose, center_from_corners, center_midpoint, center_separation, REFERENCE_CENTER_DISTANCE_M
from .calibration import load_calibration
from .web import Dashboard

class TagNode(Node):
    def __init__(self, cameras, hz, dashboard=None):
        super().__init__('tianji_apriltag')
        self.bridge, self.detector = CvBridge(), Detector()
        self.calibration = load_calibration()
        self.last, self.received = {}, set()
        self.hz = hz
        self.dashboard = dashboard
        self.outputs, self.images = {}, {}
        for camera in cameras:
            prefix = f'/perception/sensors/hik_camera_{camera}'
            self.outputs[camera] = self.create_publisher(String, f'/apriltag/camera_{camera}/detections', 10)
            self.images[camera] = self.create_publisher(Image, f'/apriltag/camera_{camera}/annotated', qos_profile_sensor_data)
            self.create_subscription(Image, prefix+'/image', lambda msg, c=camera: self.process(c, msg), qos_profile_sensor_data)
        self.cameras = cameras
        self.create_timer(5.0, self.status)
        self.get_logger().info(f'Listening to cameras {cameras}; tag36h11 ID18=34mm ID19=24mm')

    def status(self):
        for c in self.cameras:
            if c not in self.received:
                self.get_logger().warning(f'camera {c}: waiting for image topic')

    def process(self, camera, msg):
        self.received.add(camera)
        now = time.monotonic()
        if now - self.last.get(camera, 0) < 1 / self.hz:
            return
        self.last[camera] = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            info = self.calibration.get(camera)
            if info is not None and image.shape[:2] != (info.height, info.width):
                raise ValueError(f'camera {camera}: image {image.shape[1]}x{image.shape[0]} differs from pinned calibration {info.width}x{info.height}')
            tags = self.detector.detect(image)
            for tag in tags:
                tag['center_px'] = center_from_corners(tag['corners_px'])
                info = self.calibration.get(camera)
                try:
                    if info is None:
                        raise ValueError('No pinned calibration for this camera')
                    tag.update(estimate_pose(tag['corners_px'], tag['size_m'], info, image.shape))
                except ValueError as exc:
                    tag['pose_error'] = str(exc)
                points = np.asarray(tag['corners_px'], dtype=np.int32)
                cv2.polylines(image, [points], True, (0, 255, 0), 2)
                label = f"ID {tag['id']}"
                label += f" {tag['distance_m']:.3f}m" if 'distance_m' in tag else ' pose unavailable'
                cv2.putText(image, label, tuple(points[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            separation = center_separation(tags)
            midpoint_m = center_midpoint(tags)
            midpoint_px = None
            by_id = {tag['id']: tag for tag in tags}
            if 18 in by_id and 19 in by_id:
                c18 = np.rint(by_id[18]['center_px']).astype(int)
                c19 = np.rint(by_id[19]['center_px']).astype(int)
                # Draw the actual center-to-center segment with a dark outline.
                cv2.line(image, tuple(c18), tuple(c19), (0, 0, 0), 7)
                cv2.line(image, tuple(c18), tuple(c19), (0, 255, 255), 3)
                if midpoint_m is not None:
                    info = self.calibration[camera]
                    projected, _ = cv2.fisheye.projectPoints(
                        np.asarray(midpoint_m, dtype=np.float64).reshape(1, 1, 3),
                        np.zeros(3), np.zeros(3),
                        np.asarray(info.k).reshape(3, 3), np.asarray(info.d))
                    midpoint_px = projected.reshape(2).tolist()
                    middle = tuple(np.rint(midpoint_px).astype(int))
                    cv2.circle(image, middle, 9, (0, 0, 0), -1)
                    cv2.circle(image, middle, 6, (0, 255, 255), -1)
                    cv2.circle(image, middle, 2, (0, 0, 0), -1)
                    label = f'MID depth {midpoint_m[2]:.3f}m | span {separation * 1000:.1f}mm'
                    origin = (middle[0] + 14, middle[1])
                    (text_width, text_height), baseline = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(image, (origin[0] - 5, origin[1] - text_height - 5),
                                  (origin[0] + text_width + 5, origin[1] + baseline + 5),
                                  (0, 0, 0), -1)
                    cv2.putText(image, label, origin, cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 255), 2)
            for tag in tags:
                center = tuple(np.rint(tag['center_px']).astype(int))
                cv2.drawMarker(image, center, (255, 255, 255), cv2.MARKER_CROSS, 22, 4)
                cv2.drawMarker(image, center, (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            result = dict(camera=camera, frame_id=msg.header.frame_id, calibration='pinned-raw-fisheye-v1',
                          stamp=dict(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec), tags=tags)
            if separation is not None:
                result['tag_center_distance_m'] = separation
                result['reference_center_distance_m'] = REFERENCE_CENTER_DISTANCE_M
                result['tag_pair_midpoint_m'] = midpoint_m
                result['tag_pair_midpoint_depth_m'] = midpoint_m[2]
                result['tag_pair_midpoint_px'] = midpoint_px
            self.outputs[camera].publish(String(data=json.dumps(result, allow_nan=False)))
            annotated = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            annotated.header = msg.header
            self.images[camera].publish(annotated)
            if self.dashboard:
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    self.dashboard.update(camera, result, jpeg.tobytes())
        except Exception as exc:
            if self.dashboard:
                self.dashboard.error(camera, str(exc))
            self.get_logger().error(f'camera {camera}: {exc}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cameras', default='6,7')
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=23846)
    parser.add_argument('--no-ui', action='store_true')
    args, ros_args = parser.parse_known_args()
    if args.hz <= 0:
        parser.error('--hz must be positive')
    cameras = list(dict.fromkeys(int(c) for c in args.cameras.split(',')))
    host = args.host
    if 'HARBOR_WEBVIEW_LOCALHOST_ONLY' in os.environ:
        host = '127.0.0.1' if os.environ['HARBOR_WEBVIEW_LOCALHOST_ONLY'].lower() == 'true' else '0.0.0.0'
    port = int(os.environ.get('HARBOR_WEBVIEW_INTERFACE_PORT', args.port))
    rclpy.init(args=ros_args)
    dashboard = None
    node = None
    try:
        dashboard = None if args.no_ui else Dashboard(cameras, host, port)
        node = TagNode(cameras, args.hz, dashboard)
        if dashboard:
            node.get_logger().info(f'AprilTag UI: http://{host}:{port}/')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if dashboard:
            dashboard.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
