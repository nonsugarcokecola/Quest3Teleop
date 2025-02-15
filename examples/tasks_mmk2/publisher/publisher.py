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
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 0.2
        # msg.position.y = 0.1
        # msg.position.z = -0.224
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")

        publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        msg = Pose()
        msg.position.x = 0.3
        msg.position.y = 0.1
        msg.position.z = -0.226
        publisher.publish(msg)
        self.get_logger().info(
            f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")


        # # 调整机器人头部角度
        # publisher = self.create_publisher(Float32, "mmk2_head", 10)
        # msg = Float32()
        # msg.data = 0.8
        # self.get_logger().info(f"MockPublisher published head data {msg.data}")
        # publisher.publish(msg)
        # time.sleep(2)
        #input("Press Enter to continue...")

        # # 降高度
        # for i in range(15):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = -0.15
        #     self.get_logger().info(f"MockPublisher published platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.2)
        # # input("Press Enter to continue...")
        #
        # # 松开夹抓
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_left_gripper_release", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     self.get_logger().info(f"MockPublisher published mmk2_left_gripper_release data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # # 绝对距离版本
        # # # 伸到碗前
        # # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.92
        # # msg.position.y = -1.07
        # # msg.position.z = 0.849
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # # 相对距离版本
        # # # 伸到碗前
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 0.347
        # msg.position.y = 0.01
        # msg.position.z = -0.224
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        # #input("Press Enter to continue...")
        #
        # # # 绝对距离版本
        # # # 伸到碗壁
        # # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.92
        # # msg.position.y = -1.124
        # # msg.position.z = 0.799
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # 相对距离版本
        # # 伸到碗壁
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 0.054
        # msg.position.y = 0.001
        # msg.position.z = -0.05
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        # # input("Press Enter to continue...")
        #
        # # 抓住碗壁
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_left_gripper_tighten", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     self.get_logger().info(f"MockPublisher published mmk2_left_gripper_tighten data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # 提起来碗
        # for i in range(15):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = 0.2
        #     self.get_logger().info(f"MockPublisher published mmk2_platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.2)
        # #input("Press Enter to continue...")
        #
        # # # 绝对距离版本
        # # # 把碗放到托盘上空
        # # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.72
        # # msg.position.y = -1.0
        # # msg.position.z = 1.002
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # 绝对距离版本
        # # 把碗放到托盘上空
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = -0.134
        # msg.position.y = -0.201
        # msg.position.z = 0.203
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        # # input("Press Enter to continue...")
        #
        #
        # # 下降高度
        # for i in range(16):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = -0.18
        #     self.get_logger().info(f"MockPublisher published mmk2_platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # 松开碗壁
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_left_gripper_release", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     self.get_logger().info(f"MockPublisher published mmk2_left_gripper_release data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # 上升高度
        # for i in range(6):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = 0.1
        #     self.get_logger().info(f"MockPublisher published mmk2_platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # # 绝对位置
        # # # 移开手臂
        # # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.923
        # # msg.position.y = -1.008
        # # msg.position.z = 1.002
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # 相对位置版本
        # # 移开手臂
        # publisher = self.create_publisher(Pose, "mmk2_left_end", 10)
        # msg = Pose()
        # msg.position.x = 0.018
        # msg.position.y = 0.202
        # msg.position.z = -0.0
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        #
        # # input("Press Enter to continue...")
        # # 松开夹抓
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_right_gripper_release", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     self.get_logger().info(f"MockPublisher published mmk2_right_gripper_release data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # # 绝对位置版本
        # # # 伸到猕猴桃上
        # # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.521
        # # msg.position.y = -1.0
        # # msg.position.z =  0.846
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # 相对位置版本
        # # 伸到猕猴桃上
        # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # msg = Pose()
        # msg.position.x = 0.278
        # msg.position.y = 0.03
        # msg.position.z = -0.226
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_right_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        # # input("Press Enter to continue...")
        #
        # # # 绝对位置版本
        # # # 伸到猕猴桃前
        # # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.559
        # # msg.position.y = -1.1
        # # msg.position.z = 0.886
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # 相对位置版本
        # # 伸到猕猴桃前
        # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # msg = Pose()
        # msg.position.x = 0.101
        # msg.position.y = 0.039
        # msg.position.z = 0.04
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_left_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        # # input("Press Enter to continue...")
        #
        # # 降高度
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = -0.2
        #     self.get_logger().info(f"MockPublisher published platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.2)
        # #input("Press Enter to continue...")
        #
        # # 抓住猕猴桃
        # for i in range(5):
        #     publisher = self.create_publisher(Float32, "mmk2_right_gripper_tighten", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     self.get_logger().info(f"MockPublisher published mmk2_right_gripper_tighten data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")
        #
        # # 升起
        # for i in range(20):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = 10.0
        #     self.get_logger().info(f"MockPublisher published platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(1)
        # #input("Press Enter to continue...")
        #
        # # # 绝对位置版本
        # # # 移动到碗的上空
        # # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # # msg = Pose()
        # # msg.position.x = 0.741
        # # msg.position.y = -1.042
        # # msg.position.z = 0.962
        # # publisher.publish(msg)
        # # self.get_logger().info(
        # #     f"mmk2_right_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # # time.sleep(2)
        # # #input("Press Enter to continue...")
        #
        # # 绝对位置版本
        # # 移动到碗的上空
        # publisher = self.create_publisher(Pose, "mmk2_right_end", 10)
        # msg = Pose()
        # msg.position.x = -0.058
        # msg.position.y = 0.183
        # msg.position.z = 0.076
        # publisher.publish(msg)
        # self.get_logger().info(
        #     f"mmk2_right_end 发布端位置: ({msg.position.x}, {msg.position.y}, {msg.position.z})")
        # time.sleep(2)
        # # input("Press Enter to continue...")
        #
        # # 升起
        # for i in range(5):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = -0.05
        #     self.get_logger().info(f"MockPublisher published platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.2)
        # #input("Press Enter to continue...")
        #
        # # 松开猕猴桃
        # for i in range(10):
        #     publisher = self.create_publisher(Float32, "mmk2_right_gripper_release", 10)
        #     msg = Float32()
        #     msg.data = 1.0
        #     self.get_logger().info(f"MockPublisher published mmk2_right_gripper_release data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.)
        # #input("Press Enter to continue...")
        #
        # # 升起
        # for i in range(7):
        #     publisher = self.create_publisher(Float32, "mmk2_platform", 10)
        #     msg = Float32()
        #     msg.data = 0.05
        #     self.get_logger().info(f"MockPublisher published platform data {msg.data}")
        #     publisher.publish(msg)
        #     time.sleep(0.1)
        # #input("Press Enter to continue...")

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    node.publish_messages()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()