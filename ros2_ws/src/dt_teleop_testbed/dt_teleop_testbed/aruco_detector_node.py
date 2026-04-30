#!/usr/bin/env python3
"""
ArUco detection node — dt_teleop_testbed.

Subscribes to RealSense RGB + depth + camera_info, detects 4x4 ArUco
markers in the image, and publishes each detected marker's 3D pose
relative to the camera frame.

For the paper's K-AoI_obj measurement, we place a single ArUco marker
on a "brick" in the robot's workspace. The digital twin (Unity) gets
told where the brick is via a ROS topic. Under network impairment
(netem), the twin's brick lags behind the real brick — and we measure
this lag via K-AoI weighted by the brick's instantaneous velocity.

Published topics:
    /aruco/poses         geometry_msgs/PoseArray (all detected markers)
    /aruco/marker_0_pose geometry_msgs/PoseStamped (for marker id 0)
    /aruco/marker_0_joint_states sensor_msgs/JointState (x,y,z as "joints"
                                 so the existing aoi_logger can pick
                                 it up as a generic pose-tracked entity)

Subscribed topics:
    /camera/color/image_raw         sensor_msgs/Image
    /camera/color/camera_info       sensor_msgs/CameraInfo

Parameters:
    marker_size_m       0.05   side length of the ArUco marker in meters
    dictionary          "4X4_50"  OpenCV aruco dictionary name
    target_marker_id    0       which marker is the "brick"
    publish_rate_hz     30.0    max publish rate
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseArray, PoseStamped, Pose

try:
    import cv2
    import cv2.aruco as aruco
except ImportError as exc:
    raise SystemExit(
        "This node requires opencv-python and opencv-contrib-python. "
        "Install with: pip install opencv-contrib-python") from exc


_ARUCO_DICTS = {
    "4X4_50": aruco.DICT_4X4_50,
    "4X4_100": aruco.DICT_4X4_100,
    "5X5_50": aruco.DICT_5X5_50,
    "6X6_50": aruco.DICT_6X6_50,
    "7X7_50": aruco.DICT_7X7_50,
}


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector")

        self.declare_parameter("marker_size_m", 0.02)
        self.declare_parameter("dictionary", "4X4_50")
        self.declare_parameter("target_marker_id", 0)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic",
                               "/camera/color/camera_info")

        self.marker_size = float(self.get_parameter("marker_size_m").value)
        dict_name = self.get_parameter("dictionary").value
        self.target_id = int(self.get_parameter("target_marker_id").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value

        if dict_name not in _ARUCO_DICTS:
            raise ValueError(
                f"Unknown dictionary '{dict_name}'. "
                f"Valid: {list(_ARUCO_DICTS)}")

        self.aruco_dict = aruco.getPredefinedDictionary(
            _ARUCO_DICTS[dict_name])
        self.aruco_params = aruco.DetectorParameters()
        # OpenCV 4.7+ API
        if hasattr(aruco, "ArucoDetector"):
            self.detector = aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params)
        else:
            self.detector = None  # fall back to aruco.detectMarkers

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self._last_publish_t = 0.0

        self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, 10)
        self.create_subscription(
            Image, image_topic, self._on_image, 10)

        self.pub_poses = self.create_publisher(
            PoseArray, "/aruco/poses", 10)
        self.pub_target = self.create_publisher(
            PoseStamped,
            f"/aruco/marker_{self.target_id}_pose", 10)
        # JointState-style publish so the existing aoi_logger can be
        # reconfigured to log object AoI without adding a new subscriber
        self.pub_js = self.create_publisher(
            JointState,
            f"/aruco/marker_{self.target_id}_joint_states", 10)

        self.get_logger().info(
            f"ArucoDetectorNode ready\n"
            f"  dictionary = {dict_name}\n"
            f"  marker_size = {self.marker_size} m\n"
            f"  target_id = {self.target_id}\n"
            f"  max rate = {self.rate_hz} Hz")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_matrix is None:
            K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
            D = np.asarray(msg.d, dtype=np.float64)
            self.camera_matrix = K
            self.dist_coeffs = D
            self.get_logger().info(
                f"Camera calibrated: fx={K[0,0]:.1f} fy={K[1,1]:.1f}")

    def _on_image(self, msg: Image) -> None:
        if self.camera_matrix is None:
            return

        now = time.time()
        if (now - self._last_publish_t) < (1.0 / self.rate_hz):
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"cv_bridge error: {exc}")
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            return

        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size,
            self.camera_matrix, self.dist_coeffs)

        pose_array = PoseArray()
        pose_array.header = msg.header  # preserves camera frame + stamp

        target_pose: Optional[Pose] = None

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            # Convert rotation vector to quaternion
            R, _ = cv2.Rodrigues(rvec)
            qw, qx, qy, qz = self._rotation_matrix_to_quat(R)

            p = Pose()
            p.position.x = float(tvec[0])
            p.position.y = float(tvec[1])
            p.position.z = float(tvec[2])
            p.orientation.w = float(qw)
            p.orientation.x = float(qx)
            p.orientation.y = float(qy)
            p.orientation.z = float(qz)

            pose_array.poses.append(p)
            if int(marker_id) == self.target_id:
                target_pose = p

        self.pub_poses.publish(pose_array)

        if target_pose is not None:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = target_pose
            self.pub_target.publish(ps)

            # Also publish as JointState for aoi_logger compatibility:
            # treat (x, y, z) as 3 "joints" so logger's existing
            # pipeline works without modification.
            js = JointState()
            js.header = msg.header
            js.name = ["brick_x", "brick_y", "brick_z"]
            js.position = [
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
            ]
            self.pub_js.publish(js)

        self._last_publish_t = now

    @staticmethod
    def _rotation_matrix_to_quat(R: np.ndarray):
        """Convert a 3x3 rotation matrix to (qw, qx, qy, qz)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return qw, qx, qy, qz


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
