import rclpy
import traceback

import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from discoverse.envs.mmk2_base import MMK2Cfg
from discoverse.examples.tasks_mmk2.pick_kiwi import SimNode
from discoverse.task_base import MMK2TaskBase

from discoverse.examples.tasks_mmk2.config.constants import TIGHTEN_THRESHOLD, RELEASE_THRESHOLD
from discoverse.examples.tasks_mmk2.config.constants import TIGHTEN_WEIGHT, RELEASE_WEIGHT
from discoverse.examples.tasks_mmk2.config.constants import UP_WEIGHT, DOWN_WEIGHT
from discoverse.examples.tasks_mmk2.config.constants import ARM_SCALE
from discoverse.examples.tasks_mmk2.config.constants import LINEAR_THRESHOLD, ANGULAR_THRESHOLD
from discoverse.examples.tasks_mmk2.config.constants import LINEAR_VELOCITY, ANGULAR_VELOCITY

class SimNode(MMK2TaskBase):
    def domain_randomization(self):
        # 随机 杯子位置
        self.mj_data.qpos[self.njq + 0] += 2. * (np.random.random() - 0.5) * 0.05
        self.mj_data.qpos[self.njq + 1] += 2. * (np.random.random() - 0.5) * 0.025

        # 随机 盘子位置
        wood_y_bios = (np.random.random() - 0.75) * 0.05
        self.mj_data.qpos[self.njq + 7 + 0] += 2. * (np.random.random() - 0.5) * 0.05
        self.mj_data.qpos[self.njq + 7 + 1] += wood_y_bios
        self.mj_data.qpos[self.njq + 7 + 2] += 0.01

        # 随机 木盘位置
        self.mj_data.qpos[self.njq + 7 * 2 + 0] += 2. * (np.random.random() - 0.5) * 0.05
        self.mj_data.qpos[self.njq + 7 * 2 + 1] += wood_y_bios

        # 随机 杯盖位置
        self.mj_data.qpos[self.njq + 7 * 3 + 0] += 2. * (np.random.random() - 0.5) * 0.1
        self.mj_data.qpos[self.njq + 7 * 3 + 1] += 2. * (np.random.random() - 0.5) * 0.02

    def check_success(self):
        return 1


class MMK2TwistSubscriber(Node):
    def __init__(self, sim_node):
        super().__init__('action_subscriber')

        self.absolute = False
        self.sim_node = sim_node
        self.left_arm_pre = None
        self.right_arm_pre = None
        # 订阅话题
        self.mmk2_topics = [
            "mmk2_linear", "mmk2_angular", #  Float32 类型,
        ]

        # 设置定时器，每10秒检查是否接收到消息
        self.timer = self.create_timer(5.0, self.check_timeout)

        self.last_received_time = self.get_clock().now()

        # 存储订阅对象
        self.mmk2_subscriptions = {}

        for topic_name in self.mmk2_topics:
            msg_type = self.get_message_type(topic_name)  # 获取消息类型
            if msg_type:
                self.mmk2_subscriptions[topic_name] = self.create_subscription(
                    msg_type, topic_name, lambda msg, t=topic_name: self.update_simulation(msg, t), 10
                )

        self.get_logger().info("SimNodeUpdater initialized, waiting for the publishers to come online...")

    def get_message_type(self, topic_name):
        if "end" in topic_name:  # mmk2_left_end, mmk2_right_end
            return Pose
        elif "gripper" in topic_name or ("head" or "platform" in topic_name):  # mmk2_left_gripper, mmk2_right_gripper
            return Float32
        else:
            self.get_logger().warn(f"Unknown Topic name: {topic_name}")
            return None

    def update_simulation(self, msg, topic_name):
        try:
            if "linear" in topic_name:
                # self.get_logger().info(f"********************{topic_name} 尝试更新sim_ndoe线速度分量, 分量{msg.data}*****************")
                weight = 0
                if msg.data > LINEAR_THRESHOLD:
                    weight = LINEAR_VELOCITY
                elif -LINEAR_THRESHOLD > msg.data:
                    weight = -LINEAR_VELOCITY
                self.sim_node.tctr_base[0] = weight

                if msg.data != 0:
                    self.get_logger().info(f"{topic_name} 更新sim_node线速度分量，摇杆分量{msg.data}")
            elif "angular" in topic_name:
                # self.get_logger().info(
                    # f"********************{topic_name} 尝试更新sim_ndoe角速度分量, 分量{msg.data}*****************")
                weight = 0
                if msg.data > ANGULAR_THRESHOLD:
                    weight = -ANGULAR_VELOCITY
                elif -ANGULAR_THRESHOLD > msg.data:
                    weight = ANGULAR_VELOCITY
                self.sim_node.tctr_base[1] = weight
                if msg.data != 0:
                    self.get_logger().info(f"{topic_name} 更新sim_node角速度分量，摇杆分量{msg.data}")
            else:
                self.get_logger().warn(f"[{topic_name}] Unknown topic: {topic_name}")
        except Exception as e:
            self.get_logger().error(f"[{topic_name}] Exception: {e}")
            traceback.print_exc()  # 打印堆栈跟踪

    def check_timeout(self):
        # 检查自上次接收到消息是否超过10秒
        if (self.get_clock().now() - self.last_received_time).nanoseconds / 1e9 > 5.0:
            # 如果超过1秒没有收到消息，将变量重置为None
            self._var1 = None
            self._var2 = None
            self.get_logger().info("No message received in 1 second. Variables reset.")


def main(args=None):
    cfg = MMK2Cfg()
    cfg.use_gaussian_renderer = False
    cfg.init_key = "pick"
    cfg.gs_model_dict["bowl_yellow"] = "object/yellow_bowl.ply"
    cfg.gs_model_dict["wood"] = "object/wood.ply"
    cfg.gs_model_dict["kiwi"] = "object/kiwi.ply"
    cfg.gs_model_dict["background"] = "scene/tsimf_library_0/environment.ply"

    cfg.mjcf_file_path = "mjcf/tasks_mmk2/pick_kiwi.xml"
    cfg.obj_list = ["bowl_yellow", "wood", "kiwi"]
    cfg.sync = False
    cfg.headless = False
    cfg.render_set = {
        "fps": 25,
        "width": 640,
        "height": 480
    }
    cfg.obs_rgb_cam_id = [0, 1, 2]
    cfg.save_mjb_and_task_config = False

    rclpy.init(args=args)
    sim_node = SimNode(cfg)

    node = MMK2TwistSubscriber(sim_node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()