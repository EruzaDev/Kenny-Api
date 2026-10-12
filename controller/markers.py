"""Calibrated RGB ArUco poses for live landmark refinement (no RGB/depth registration)."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from .sensors import rotation


def marker_pose(corners, size, camera_matrix, distortion, maximum_error=2.):
    import cv2
    half = size / 2
    square = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    ok, vectors, translations, errors = cv2.solvePnPGeneric(square, np.asarray(corners, dtype=np.float64).reshape(4, 2),
        camera_matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    ranked = sorted(zip(errors.ravel(), vectors, translations), key=lambda item: item[0])
    error, vector, translation = ranked[0]
    if not math.isfinite(float(error)) or error > maximum_error or translation[2, 0] <= 0:
        return None
    orientation, _ = cv2.Rodrigues(vector)
    return translation.ravel(), orientation


def main():
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', type=Path, required=True)
    parser.add_argument('--frame', default='slam_map')
    parser.add_argument('--image', default='/camera/color/image_raw')
    parser.add_argument('--camera-info', default='/camera/color/camera_info')
    args, ros_args = parser.parse_known_args()
    environment = json.loads(args.environment.read_text())
    markers = {m['id']: m for m in environment['markers']}
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, environment['dictionary']))
    detector = cv2.aruco.ArucoDetector(dictionary)
    rclpy.init(args=ros_args)

    class Detector(Node):
        def __init__(self):
            super().__init__('kenny_marker_detector')
            self.camera = None
            self.cv = CvBridge()
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.pub = self.create_publisher(MarkerArray, '/kenny/marker_observations', 10)
            self.create_subscription(CameraInfo, args.camera_info, self.intrinsics, qos_profile_sensor_data)
            self.create_subscription(Image, args.image, self.image, qos_profile_sensor_data)

        def intrinsics(self, msg):
            if msg.k[0] > 0 and msg.k[4] > 0 and np.isfinite(msg.k).all():
                self.camera = msg

        def image(self, msg):
            if self.camera is None or self.camera.header.frame_id != msg.header.frame_id or self.camera.width != msg.width or self.camera.height != msg.height:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if not -.1 <= self.get_clock().now().nanoseconds / 1e9 - stamp <= .5:
                return
            try:
                image = self.cv.imgmsg_to_cv2(msg, desired_encoding='mono8')
                corners, ids, _ = detector.detectMarkers(image)
                if ids is None:
                    return
                tf = self.buffer.lookup_transform(args.frame, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
            except (TransformException, ValueError):
                return
            q, p = tf.transform.rotation, tf.transform.translation
            camera_rotation = rotation([q.x, q.y, q.z, q.w])
            camera_matrix = np.asarray(self.camera.k).reshape(3, 3)
            output = MarkerArray()
            for image_corners, marker_id in zip(corners, ids.ravel()):
                if int(marker_id) not in markers:
                    continue
                reference = markers[int(marker_id)]
                result = marker_pose(image_corners, reference['size'], camera_matrix, np.asarray(self.camera.d))
                if result is None:
                    continue
                translation, marker_rotation = result
                if np.linalg.norm(translation) > 2.5:
                    continue
                center = camera_rotation @ translation + [p.x, p.y, p.z]
                orientation = camera_rotation @ marker_rotation
                direction = orientation[:, 2] if reference['mount'] == 'wall' else orientation[:, 1]
                if np.linalg.norm(direction[:2]) < .5:
                    continue
                heading = math.atan2(direction[1], direction[0]) - (math.pi / 2 if reference['mount'] == 'floor' else 0.)
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = args.frame, msg.header.stamp
                marker.id, marker.ns, marker.type, marker.action = int(marker_id), 'observed_aruco', Marker.CUBE, Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, center)
                marker.pose.orientation.z, marker.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
                marker.scale.x = marker.scale.y = reference['size']
                marker.scale.z = .01
                output.markers.append(marker)
            self.pub.publish(output)

    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
