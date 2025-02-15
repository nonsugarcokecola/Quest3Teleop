import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import cv2

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')
        self.cnt = 0
        self.bridge = CvBridge()

        self.cam_publishers = {
            "left_cam": self.create_publisher(CompressedImage, '/left_cam/compressed', 10),
            "mid_cam": self.create_publisher(CompressedImage, '/mid_cam/compressed', 10),
            "right_cam": self.create_publisher(CompressedImage, '/right_cam/compressed', 10),
        }

        self.timer = self.create_timer(0.01, self.publish_images)

        self.images = {
            "left_cam": np.zeros((480, 640, 3), dtype=np.uint8),
            "mid_cam": np.zeros((480, 640, 3), dtype=np.uint8),
            "right_cam": np.zeros((480, 640, 3), dtype=np.uint8),
        }

    def publish_images(self):
        for cam, publisher in self.cam_publishers.items():
            rgb_image = cv2.cvtColor(self.images[cam], cv2.COLOR_BGR2RGB)
            success, encoded_image = cv2.imencode('.jpg', rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if success:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = encoded_image.tobytes()
                publisher.publish(msg)

    def update_image(self, cam_id, np_array):
        """
        更新某个摄像头的图像数据
        :param cam_id: 'left_cam', 'mid_cam', 'right_cam'
        :param np_array: 形状 (480, 640, 3) 的 numpy 数组
        """
        if cam_id in self.images and np_array.shape == (480, 640, 3):
            self.images[cam_id] = np_array
        else:
            self.get_logger().warn(f"Invalid camera ID or image shape for {cam_id}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
