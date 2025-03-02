import threading
import concurrent.futures

import rclpy
import traceback

import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float32

from discoverse.envs.mmk2_base import MMK2Cfg
from discoverse.examples.tasks_mmk2.pick_kiwi import SimNode
from discoverse.task_base import MMK2TaskBase

from mmk2_types.types import MMK2Components
from discoverse.examples.tasks_mmk2.utils.utils import control_spine_servo_separate

from discoverse.examples.tasks_mmk2.config.constants import UP_WEIGHT, DOWN_WEIGHT
from mmk2_types.grpc_msgs import (
    JointState,
)

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

class SimPlatformSubscriber(Node):
    def __init__(self, sim_node):
        super().__init__('sim_platform_subscriber')

        self.sim_node = sim_node
        # 订阅话题
        self.mmk2_platform_topic = "mmk2_platform"
        # 存储订阅对象
        self.mmk2_subscriptions = {}

        msg_type = Float32
        self.mmk2_subscriptions[self.mmk2_platform_topic] = self.create_subscription(
            msg_type, self.mmk2_platform_topic, lambda msg, t=self.mmk2_platform_topic: self.update_simulation(msg, t), 10
        )

        self.get_logger().info("SimNodeUpdater initialized, waiting for the publishers to come online...")

    def update_simulation(self, msg, topic_name):
        try:
            weight = None
            if msg.data > 0:
                weight = UP_WEIGHT
            elif msg.data < 0:
                weight = DOWN_WEIGHT
            if weight is not None:
                new_height = self.sim_node.tctr_slide[0] + weight
                if new_height > 1:
                    new_height = 1
                    self.get_logger().warn(
                        f"[{topic_name}] tctr_slide[0] exceeded upper threshold: {new_height}")
                elif new_height < 0:
                    new_height = 0
                    self.get_logger().warn(
                        f"[{topic_name}] tctr_slide[0] exceeded lower threshold: {new_height}")
                self.sim_node.tctr_slide[0] = new_height

                self.get_logger().info(f"[{topic_name}] 更新 sim_node: 平台位置 x {self.sim_node.tctr_slide[0]}")
        except Exception as e:
            self.get_logger().error(f"[{topic_name}] Exception: {e}")
            traceback.print_exc()  # 打印堆栈跟踪
class MMK2PlatformSubscriber(Node):
    def __init__(self, mmk2, trgt_joint_action):
        super().__init__('mmk2_platform_subscriber')

        self.mmk2 = mmk2
        self.trgt_joint_action = trgt_joint_action
        # 订阅话题
        self.mmk2_platform_topic = "mmk2_platform"
        # 存储订阅对象
        self.platform_subscriptions = self.create_subscription(
            Float32,
            self.mmk2_platform_topic,
            self.update_simulation,
            10
        )

        self.get_logger().info("MMK2 Platform已完成初始化，等待指令")

    def update_simulation(self, msg):
        try:
            weight = None
            if msg.data > 0:
                weight = UP_WEIGHT
            elif msg.data < 0:
                weight = DOWN_WEIGHT
            if weight is not None:
                new_height = self.trgt_joint_action[MMK2Components.SPINE].position[0]  + weight
                if new_height > 1:
                    new_height = 1
                    self.get_logger().warn(
                        f"设定的高度: {new_height}, 超过了MMK2的最高值")
                elif new_height < 0:
                    new_height = 0
                    self.get_logger().warn(
                        f"设定的高度{new_height}， 超过了MMK2的最低值")
                self.trgt_joint_action[MMK2Components.SPINE] = JointState(position=[new_height])

                control_spine_servo_separate(self.mmk2, self.trgt_joint_action)
                self.get_logger().info(f"{self.mmk2_platform_topic} 尝试更新 MMk2: 至平台位置 x {self.trgt_joint_action[MMK2Components.SPINE]}")

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            traceback.print_exc()  # 打印堆栈跟踪


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

    node = SimPlatformSubscriber(sim_node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()