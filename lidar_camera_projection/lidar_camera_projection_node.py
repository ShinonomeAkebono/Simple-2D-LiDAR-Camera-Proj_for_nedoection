import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
import tf2_py as tf2

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from .utils.math import concatenate_translation, quaternion_to_rotation_matrix

class LiDARCameraProjectionNode(Node):

    def __init__(self):
        super().__init__('lidar_camera_projection_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.image_subscriber_ = self.create_subscription(Image, '/sort_system/capture', self.image_callback, qos_profile)
        self.lidar_subscriber_ = self.create_subscription(PointCloud2, '/aw_points', self.scan_callback, qos_profile)
        self.timer_ = self.create_timer(0.03, self.projection_callback)
        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_scan = None

        self.base_tf_name = "base_link"
        self.camera_tf_name = "camera_link"
        self.camera_optical_tf_name = "camera_rgb_optical_frame"
        self.LiDAR_tf_name = "base_scan"

        self.camera_intrinsic_matrix = np.array([
            1696.80268, 0.0, 960.5,
            0.0, 1696.80268, 540.5,
            0.0, 0.0, 1.0
        ]).reshape(3, 3)
        
        self.initialize = False

        self.T_camera_LIDAR = None
        self.projection_matrix = None

    def image_callback(self, image_msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        self.latest_image = cv_image

    def scan_callback(self, scan_msg: PointCloud2):
        self.latest_scan = scan_msg

    def lookup_transform(self, target_frame, source_frame):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1))
            return transform
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            self.get_logger().warning('Could not transform %s to %s: %s' % (source_frame, target_frame, str(e)))
            return None

    def projection_callback(self):
        if not self.initialize:
            camera_lidar_transform = self.lookup_transform(self.camera_tf_name, self.LiDAR_tf_name)
            camera_base_transform = self.lookup_transform(self.camera_optical_tf_name, self.base_tf_name)

            if camera_lidar_transform:
                translation = camera_lidar_transform.transform.translation
                quaternion = camera_lidar_transform.transform.rotation

                self.T_LiDAR_camera = concatenate_translation(
                    quaternion_to_rotation_matrix(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    np.array([translation.x, translation.y, translation.z])
                )
            if camera_base_transform:
                translation = camera_base_transform.transform.translation
                quaternion = camera_base_transform.transform.rotation

                T_camera_base = concatenate_translation(
                    quaternion_to_rotation_matrix(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    np.array([translation.x, translation.y, translation.z])
                )
                T_camera_base[0, 3] = 0
                T_camera_base[1, 3] = 0
                T_camera_base[2, 3] = 0
                self.projection_matrix = self.camera_intrinsic_matrix @ T_camera_base[:3, :4]

            if not camera_lidar_transform is None and not camera_base_transform is None:
                self.initialize = True
        
        if self.initialize and not self.latest_image is None and not self.latest_scan is None:
            debug_image = self.latest_image.copy()

            for point in pc2.read_points(self.latest_scan, skip_nans=True):
                P_scan = np.array([point[0], point[1], point[2], 1.0])
                P_camera = self.T_LiDAR_camera @ P_scan

                if P_camera[0] < 0:
                    continue
                P_image = self.projection_matrix @ P_camera

                image_x = P_image[0] / P_image[2]
                image_y = P_image[1] / P_image[2]

                if 0 <= image_x <= self.latest_image.shape[1] and 0 <= image_y <= self.latest_image.shape[0]:
                    circle_r = 6 - int(np.linalg.norm(P_scan) / 3.0 * 3)
                    cv2.circle(debug_image, (int(image_x), int(image_y)), circle_r, (0, 0, 255), -1)

            resize_image = cv2.resize(debug_image, (debug_image.shape[1] // 2, debug_image.shape[0] // 2))
            cv2.imshow('Camera Image', resize_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    lidar_camera_projection_node = LiDARCameraProjectionNode()

    rclpy.spin(lidar_camera_projection_node)

    lidar_camera_projection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
