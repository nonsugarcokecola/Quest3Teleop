import os
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from discoverse.mmk2 import MMK2FIK
from discoverse import DISCOVERSE_ASSERT_DIR
from discoverse.airbot_play import AirbotPlayFIK
from discoverse.envs.mmk2_base import MMK2Base, MMK2Cfg
from discoverse.utils.joy_stick_ros2 import JoyTeleopRos2

class MMK2JOY(MMK2Base, Node):
    arm_action_init_position = {
        "pick" : {
            "l" : np.array([0.223,  0.21, 1.07055]),
            "r" : np.array([0.223, -0.21, 1.07055]),
        },
    }

    target_control = np.zeros(19)
    def __init__(self, config: MMK2Cfg):
        self.arm_action = config.init_key
        self.tctr_base = self.target_control[:2]
        self.tctr_slide = self.target_control[2:3]
        self.tctr_head = self.target_control[3:5]
        self.tctr_left_arm = self.target_control[5:11]
        self.tctr_lft_gripper = self.target_control[11:12]
        self.tctr_right_arm = self.target_control[12:18]
        self.tctr_rgt_gripper = self.target_control[18:19]

        super().__init__(config)
        Node.__init__(self, 'MMK2_mujoco_node')

        self.mj_base = self.mj_data.qpos[:7]
        self.mj_wheel_joint = self.mj_data.qpos[7:9]
        self.mj_slide_joint = self.mj_data.qpos[9:10]
        self.mj_head_joint = self.mj_data.qpos[10:12]
        self.mj_left_arm_joint = self.mj_data.qpos[12:18]
        self.mj_left_gripper_joint = self.mj_data.qpos[18:20]
        self.mj_right_arm_joint = self.mj_data.qpos[20:26]
        self.mj_right_gripper_joint = self.mj_data.qpos[26:28]

        self.mj_base_vel = self.mj_data.qvel[:6]
        self.mj_wheel_joint_vel = self.mj_data.qvel[6:8]
        self.mj_slide_joint_vel = self.mj_data.qvel[8:9]
        self.mj_head_joint_vel = self.mj_data.qvel[9:12]
        self.mj_left_arm_joint_vel = self.mj_data.qvel[12:18]
        self.mj_left_gripper_joint_vel = self.mj_data.qvel[18:20]
        self.mj_right_arm_joint_vel = self.mj_data.qvel[20:26]
        self.mj_right_gripper_joint_vel = self.mj_data.qvel[26:28]

        self.lft_arm_target_pose = self.arm_action_init_position[self.arm_action]["l"].copy()
        self.lft_end_euler = np.zeros(3)
        self.rgt_arm_target_pose = self.arm_action_init_position[self.arm_action]["r"].copy()
        self.rgt_end_euler = np.zeros(3)

        self.arm_fik = AirbotPlayFIK(urdf = os.path.join(DISCOVERSE_ASSERT_DIR, "urdf/airbot_play_v3_gripper_fixed.urdf"))

        self.teleop = JoyTeleopRos2()
        self.sub = self.create_subscription(Joy, '/joy_throttle', self.teleop.joy_callback, 10)

    def resetState(self):
        super().resetState()
        self.target_control[:] = self.init_joint_ctrl[:]
        self.lft_arm_target_pose = self.arm_action_init_position[self.arm_action]["l"].copy()
        self.lft_end_euler = np.zeros(3)
        self.rgt_arm_target_pose = self.arm_action_init_position[self.arm_action]["r"].copy()
        self.rgt_end_euler = np.zeros(3)
        self.teleop.reset()

    def teleopProcess(self):
        linear_vel  = 0.00
        angular_vel = 1.00
        
        self.teleop.joy_cmd.buttons[4] = 1
        print("Current Teleop control buttons:\t", self.teleop.joy_cmd.buttons)
        print("Current Teleop control axes:\t", self.teleop.joy_cmd.axes)
        print("Buttons 4: \t", self.teleop.joy_cmd.buttons[4])
        print("Buttons 5: \t", self.teleop.joy_cmd.buttons[5])
        
        if self.teleop.joy_cmd.buttons[4]:   # left arm
            tmp_lft_arm_target_pose = self.lft_arm_target_pose.copy()
            tmp_lft_arm_target_pose[0] += self.teleop.joy_cmd.axes[7] * 0.1 / self.render_fps
            tmp_lft_arm_target_pose[1] += self.teleop.joy_cmd.axes[6] * 0.1 / self.render_fps
            tmp_lft_arm_target_pose[2] += self.teleop.joy_cmd.axes[1] * 0.1 / self.render_fps
            print("Tmp left arm target pose", tmp_lft_arm_target_pose)
            delta_gripper = (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * 1. / self.render_fps
            self.tctr_lft_gripper[0] += delta_gripper
            self.tctr_lft_gripper[0] = np.clip(self.tctr_lft_gripper[0], 0, 1)
            el = self.lft_end_euler.copy()
            el[0] += self.teleop.joy_cmd.axes[4] * 0.35 / self.render_fps
            el[1] += self.teleop.joy_cmd.axes[3] * 0.35 / self.render_fps
            el[2] += self.teleop.joy_cmd.axes[0] * 0.35 / self.render_fps
            try:
                self.tctr_left_arm[:] = MMK2FIK().get_armjoint_pose_wrt_footprint(tmp_lft_arm_target_pose, self.arm_action, "l", self.tctr_slide[0], self.tctr_left_arm, Rotation.from_euler('zyx', el).as_matrix())
                self.lft_arm_target_pose[:] = tmp_lft_arm_target_pose
                self.lft_end_euler[:] = el
            except ValueError:
                print("Invalid left arm target position:", tmp_lft_arm_target_pose)

        elif self.teleop.joy_cmd.buttons[5]: # right arm
            tmp_rgt_arm_target_pose = self.rgt_arm_target_pose.copy()
            tmp_rgt_arm_target_pose[0] += self.teleop.joy_cmd.axes[7] * 0.1 / self.render_fps
            tmp_rgt_arm_target_pose[1] += self.teleop.joy_cmd.axes[6] * 0.1 / self.render_fps
            tmp_rgt_arm_target_pose[2] += self.teleop.joy_cmd.axes[1] * 0.1 / self.render_fps

            delta_gripper = (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * 1. / self.render_fps
            self.tctr_rgt_gripper[0] += delta_gripper
            self.tctr_rgt_gripper[0] = np.clip(self.tctr_rgt_gripper[0], 0, 1)
            el = self.rgt_end_euler.copy()
            el[0] += self.teleop.joy_cmd.axes[4] * 0.35 / self.render_fps
            el[1] += self.teleop.joy_cmd.axes[3] * 0.35 / self.render_fps
            el[2] += self.teleop.joy_cmd.axes[0] * 0.35 / self.render_fps
            try:
                self.tctr_right_arm[:] = MMK2FIK().get_armjoint_pose_wrt_footprint(tmp_rgt_arm_target_pose, self.arm_action, "r", self.tctr_slide[0], self.tctr_right_arm, Rotation.from_euler('zyx', el).as_matrix())
                self.rgt_arm_target_pose[:] = tmp_rgt_arm_target_pose
                self.rgt_end_euler[:] = el
            except ValueError:
                print("Invalid right arm target position:", tmp_rgt_arm_target_pose)

        else:
            delta_height = (self.teleop.joy_cmd.axes[2] - self.teleop.joy_cmd.axes[5]) * 0.1 / self.render_fps
            if self.tctr_slide[0] + delta_height< self.mj_model.joint("slide_joint").range[0]:
                delta_height = self.mj_model.joint("slide_joint").range[0] - self.tctr_slide[0]
            elif self.tctr_slide[0] + delta_height > self.mj_model.joint("slide_joint").range[1]:
                delta_height = self.mj_model.joint("slide_joint").range[1] - self.tctr_slide[0]
            self.tctr_slide[0] += delta_height
            self.lft_arm_target_pose[2] -= delta_height
            self.rgt_arm_target_pose[2] -= delta_height

            self.tctr_head[0] += self.teleop.joy_cmd.axes[3] * 1. / self.render_fps
            self.tctr_head[1] -= self.teleop.joy_cmd.axes[4] * 1. / self.render_fps
            self.tctr_head[0] = np.clip(self.tctr_head[0], self.mj_model.joint("head_yaw_joint").range[0], self.mj_model.joint("head_yaw_joint").range[1])
            self.tctr_head[1] = np.clip(self.tctr_head[1], self.mj_model.joint("head_pitch_joint").range[0], self.mj_model.joint("head_pitch_joint").range[1])

            linear_vel  = 1.0 * self.teleop.joy_cmd.axes[1]**2 * np.sign(self.teleop.joy_cmd.axes[1])
            angular_vel = 2.0 * self.teleop.joy_cmd.axes[0]**2 * np.sign(self.teleop.joy_cmd.axes[0])
        self.base_move(linear_vel, angular_vel)

    def base_move(self, linear_vel, angular_vel):
        self.tctr_base[0] = linear_vel
        self.tctr_base[1] = angular_vel

    def printMessage(self):
        super().printMessage()

        print("    lta local = {}".format(self.lft_arm_target_pose))
        print("    rta local = {}".format(self.rgt_arm_target_pose))
        print("       euler  = {}".format(self.lft_end_euler))
        print("       euler  = {}".format(self.rgt_end_euler))

if __name__ == "__main__":
    rclpy.init()
    np.set_printoptions(precision=3, suppress=True, linewidth=500)

    cfg = MMK2Cfg()
    cfg.mjcf_file_path = "mjcf/tasks_mmk2/plate_coffeecup.xml"
    cfg.init_key = "pick"
    cfg.use_gaussian_renderer = False
    cfg.obs_rgb_cam_id = None
    cfg.obs_depth_cam_id = None
    cfg.render_set     = {
        "fps"    : 30,
        "width"  : 1920,
        "height" : 1080
    }

    exec_node = MMK2JOY(cfg)
    exec_node.reset()

    while exec_node.running:
        exec_node.teleopProcess()
        print("Stepping the action", exec_node.target_control)
        exec_node.step(exec_node.target_control)
        # rclpy.spin_once(exec_node)

    exec_node.destroy_node()
    rclpy.shutdown()