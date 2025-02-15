import rclpy
import traceback

import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from discoverse.envs.mmk2_base import MMK2Cfg
from discoverse.examples.tasks_mmk2.pick_kiwi import SimNode
from discoverse.task_base import MMK2TaskBase

from discoverse.examples.tasks_mmk2.config.constants import ARM_SCALE

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


class MMK2ARMSubscriber(Node):
    def __init__(self, sim_node):
        super().__init__('action_subscriber')

        self.absolute = False
        self.sim_node = sim_node
        self.left_arm_pre = None
        self.right_arm_pre = None
        # 订阅话题
        self.mmk2_topics = [
            "mmk2_left_end", "mmk2_right_end",  # POSE 类型
        ]

        # 设置定时器，每10秒检查是否接收到消息
        self.timer = self.create_timer(5.0, self.check_timeout)

        self.last_received_time = self.get_clock().now()

        # 存储订阅对象
        self.mmk2_subscriptions = {}

        for topic_name in self.mmk2_topics:
            msg_type = Pose
            if msg_type:
                self.mmk2_subscriptions[topic_name] = self.create_subscription(
                    msg_type, topic_name, lambda msg, t=topic_name: self.update_simulation(msg, t), 10
                )

        self.get_logger().info("SimNodeUpdater initialized, waiting for the publishers to come online...")

    def update_simulation(self, msg, topic_name):
        try:
            if "end" in topic_name:
                self.last_received_time = self.get_clock().now()
                target_pos = np.array([msg.position.y, -msg.position.x, -msg.position.z]) * ARM_SCALE
                target_quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

                # target_euler = Rotation.from_quat(target_quat).as_euler('zyx')
                target_quat = Rotation.from_euler('zyx', [0., -0.0551, 0.]).as_matrix()

                # Initialize the target pose for left or right arm if they are not already set
                if self.left_arm_pre is None and "left" in topic_name:
                    self.left_arm_pre = target_pos
                    print("lft arm", self.sim_node.lft_arm_target_pose)
                    print("Set the initial value of left arm", self.left_arm_pre)
                    return

                if self.right_arm_pre is None and "right" in topic_name:
                    self.right_arm_pre = target_pos
                    print("right arm", self.sim_node.rgt_arm_target_pose)
                    print("Set the initial value of right arm", self.right_arm_pre)
                    return

                if self.absolute:
                    if "left" in topic_name:
                        self.sim_node.lft_arm_target_pose[:] = self.sim_node.get_tmat_wrt_mmk2base(target_pos)
                        self.sim_node.setArmEndTarget(self.sim_node.lft_arm_target_pose, self.sim_node.arm_action, "l",
                                                     self.sim_node.sensor_lft_arm_qpos,
                                                     # Rotation.from_euler('zyx', target_euler).as_matrix())
                                                      target_quat)
                    if "right" in topic_name:
                        self.sim_node.rgt_arm_target_pose[:] = self.sim_node.get_tmat_wrt_mmk2base(target_pos)
                        self.sim_node.setArmEndTarget(self.sim_node.rgt_arm_target_pose, self.sim_node.arm_action, "r",
                                                      self.sim_node.sensor_rgt_arm_qpos,
                                                      # Rotation.from_euler('zyx', target_euler).as_matrix())
                                                      target_quat)
                else:
                    if "left" in topic_name:
                        print("lft arm current:\t", self.sim_node.lft_arm_target_pose)
                        print("Set the initial value of left arm", self.left_arm_pre)
                        # self.sim_node.lft_arm_target_pose[:] = self.sim_node.lft_arm_target_pose[:] + target_pos
                        print("lft_arm_target_pose", self.sim_node.lft_arm_target_pose[:], "left_prev", self.left_arm_pre, "offset:\t", target_pos - self.left_arm_pre)
                        self.sim_node.lft_arm_target_pose[:] = self.sim_node.lft_arm_target_pose[:] + target_pos - self.left_arm_pre
                        self.left_arm_pre = target_pos
                        self.sim_node.setArmEndTarget(self.sim_node.lft_arm_target_pose, self.sim_node.arm_action, "l",
                                                     self.sim_node.sensor_lft_arm_qpos,
                                                     # Rotation.from_euler('zyx', target_euler).as_matrix())
                                                      target_quat)
                    if "right" in topic_name:
                        # self.sim_node.rgt_arm_target_pose[:] = self.sim_node.rgt_arm_target_pose[:] + target_pos
                        print("rgt_arm_target_pose", self.sim_node.rgt_arm_target_pose, "right_prev", self.right_arm_pre, "offset:\t", target_pos - self.right_arm_pre)
                        self.sim_node.rgt_arm_target_pose[:] = self.sim_node.rgt_arm_target_pose[:] + target_pos - self.right_arm_pre
                        self.right_arm_pre = target_pos
                        self.sim_node.setArmEndTarget(self.sim_node.rgt_arm_target_pose, self.sim_node.arm_action, "r",
                                                      self.sim_node.sensor_rgt_arm_qpos,
                                                      # Rotation.from_euler('zyx', target_euler).as_matrix())
                                                      target_quat)

                self.get_logger().info(f"更新[{topic_name}]末端{self.sim_node.lft_arm_target_pose[:]}, pose given:\t {target_pos}")
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

    node = MMK2ARMSubscriber(sim_node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()