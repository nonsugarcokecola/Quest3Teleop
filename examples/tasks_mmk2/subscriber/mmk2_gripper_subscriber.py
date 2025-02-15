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


class MMK2GripperSubscriber(Node):
    def __init__(self, sim_node):
        super().__init__('action_subscriber')

        self.absolute = False
        self.sim_node = sim_node
        self.left_arm_pre = None
        self.right_arm_pre = None
        # 订阅话题
        self.mmk2_topics = [
            "mmk2_left_gripper_release", "mmk2_left_gripper_tighten",  #  Float32 类型
            "mmk2_right_gripper_release", "mmk2_right_gripper_tighten", #  Float32 类型
        ]

        # 存储订阅对象
        self.mmk2_subscriptions = {}

        for topic_name in self.mmk2_topics:
            msg_type = Float32
            if msg_type:
                self.mmk2_subscriptions[topic_name] = self.create_subscription(
                    msg_type, topic_name, lambda msg, t=topic_name: self.update_simulation(msg, t), 10
                )

        self.get_logger().info("SimNodeUpdater initialized, waiting for the publishers to come online...")

    def update_gripper(self, gripper, weight):
        new_value = gripper + weight
        if new_value > 1 or new_value < 0:
            self.get_logger().error(f"达到了夹抓位置的边界: {new_value}. 它需要在0～1之间.")
            return gripper
        return new_value

    def update_simulation(self, msg, topic_name):
        try:
            if "gripper" in topic_name:
                weight = None
                if "release" in topic_name and msg.data > RELEASE_THRESHOLD:
                    weight = RELEASE_WEIGHT
                elif "tighten" in topic_name and msg.data > TIGHTEN_THRESHOLD:
                    weight = TIGHTEN_WEIGHT
                if weight is not None:
                    if "left" in topic_name:
                        self.sim_node.tctr_lft_gripper[:] = self.update_gripper(self.sim_node.tctr_lft_gripper[:], weight)
                        self.get_logger().info(f"[{topic_name}] 更新 sim_node: 平台位置 x {msg.data}")
                    if "right" in topic_name:
                        self.sim_node.tctr_rgt_gripper[:] = self.update_gripper(self.sim_node.tctr_rgt_gripper[:], weight)
                        self.get_logger().info(f"[{topic_name}] 更新 sim_node: 平台位置 x {msg.data}")
            else:
                self.get_logger().warn(f"[{topic_name}] Unknown topic: {topic_name}")
        except Exception as e:
            self.get_logger().error(f"[{topic_name}] Exception: {e}")
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

    node = MMK2Subscriber(sim_node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()