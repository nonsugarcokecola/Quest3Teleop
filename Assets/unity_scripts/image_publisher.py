import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MultiCameraPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_publisher')

        # 创建3个图像发布者
        self.cam_publishers = {
            "left_cam": self.create_publisher(Image, '/left_cam/image_raw', 10),
            "mid_cam": self.create_publisher(Image, '/mid_cam/image_raw', 10),
            "right_cam": self.create_publisher(Image, '/right_cam/image_raw', 10),
        }

        # 定时发布，每 100ms 触发一次
        self.timer = self.create_timer(0.01, self.publish_images)
        self.bridge = CvBridge()

        # 初始化 numpy 图像数据（假设每个相机是 640x480 的 RGB 图像）
        self.images = {
            "left_cam": np.zeros((480, 640, 3), dtype=np.uint8),
            "mid_cam": np.zeros((480, 640, 3), dtype=np.uint8),
            "right_cam": np.zeros((480, 640, 3), dtype=np.uint8),
        }

    def publish_images(self):
        """ 发布所有摄像头的图像数据 """
        for cam, publisher in self.cam_publishers.items():
            img_msg = self.bridge.cv2_to_imgmsg(self.images[cam], encoding="rgb8")
            publisher.publish(img_msg)
            # self.get_logger().info(f"Published image from {cam}")

    def update_image(self, cam_id, np_array):
        """
        更新某个摄像头的图像数据
        :param cam_id: 'left_cam', 'mid_cam', 'right_cam'
        :param np_array: 形状 (480, 640, 3) 的 numpy 数组
        """
        if cam_id in self.images and np_array.shape == (480, 640, 3):
            self.images[cam_id] = np_array
            # self.get_logger().info(f"Updated image for {cam_id}")
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
