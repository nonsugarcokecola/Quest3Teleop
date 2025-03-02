import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
import time


class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')

        # 定义话题和消息类型
        self.mmk2_topics = [
            ("mmk2_left_end", Pose),
            ("mmk2_right_end", Pose),
            ("mmk2_left_gripper_release", Float32),
            ("mmk2_left_gripper_tighten", Float32),
            ("mmk2_right_gripper_release", Float32),
            ("mmk2_right_gripper_tighten", Float32),
            ("mmk2_head", Float32),
            ("mmk2_platform", Float32)
        ]

        self.get_logger().info("MockPublisher initialized.")

    def publish_messages(self):
        # # 脊柱移动测试
        # for i in range(5):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = -0.1
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        #
        # for i in range(5):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = 0.1
        #     publisher.publish(msg)
        #     time.sleep(.1)

        publisher = self.create_publisher(Float32, "mmk2_right_gripper_release", 10)
        msg = Float32()
        msg.data = 0.5
        publisher.publish(msg)

        publisher = self.create_publisher(Float32, "mmk2_right_gripper_release", 10)
        msg = Float32()
        msg.data = 0.8
        publisher.publish(msg)


        # # 夹抓测试
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_left_gripper_release", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        #
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_right_gripper_release", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        #
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_left_gripper_tighten", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        #
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_right_gripper_tighten", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        #
        # input("Press Enter to continue... for the arm operation")
        # # 机械臂控制
        # # 初始化位置
        # # 左臂
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 2.21
        # msg.position.y = 0.763
        # msg.position.z = 1.334
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # input("Press Enter to continue...")
        #
        # # 机械臂控制
        # # 初始化位置
        # # 右臂
        # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # msg = Pose()
        # msg.position.x = 2.21
        # msg.position.y = 0.763
        # msg.position.z = 1.334
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_right_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # input("Press Enter to continue...")
        #
        # # 左臂移动
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 2.21 + 0.0339 - 0.0601
        # msg.position.y = 0.763 + 0.7486 - 0.2465
        # msg.position.z = 1.334 + 0.2464 - 0.2149
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(0.5)
        # input("Press Enter to continue...")
        #
        # # 右臂移动
        # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # msg = Pose()
        # msg.position.x = 2.21 + 0.0339 -0.0596
        # msg.position.y = 0.763 + (-0.7487) - (-0.2460)
        # msg.position.z = 1.334 + 0.2472 - 0.2172
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_right_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(0.5)
        # input("Press Enter to continue...")
        #
        # # 左臂回到原位置
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 2.21
        # msg.position.y = 0.763
        # msg.position.z = 1.334
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(0.5)
        # input("Press Enter to continue...")
        #
        # # 右臂回到原位置
        # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # msg = Pose()
        # msg.position.x = 2.21
        # msg.position.y = 0.763
        # msg.position.z = 1.334
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_right_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # input("Press Enter to continue...")

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    node.publish_messages()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()