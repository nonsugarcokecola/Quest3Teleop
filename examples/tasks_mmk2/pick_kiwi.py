import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import os
import shutil
import argparse
import multiprocessing as mp
from enum import Enum

from discoverse import DISCOVERSE_ROOT_DIR
from discoverse.envs.mmk2_base import MMK2Cfg
from discoverse.task_base import MMK2TaskBase, recoder_mmk2
from discoverse.utils import get_body_tmat, step_func, SimpleStateMachine

TABLE_WIDTH_UPPER_BOUND = 1.3
TABLE_WIDTH_LOWER_BOUND = 0.2

TABLE_LENGTH_UPPER_BOUND = -0.96
TABLE_LENGTH_LOWER_BOUND = -1.46

class SimNode(MMK2TaskBase):

    def domain_randomization(self):
        self.max_trials = 1000
        self.tmp_kiwi_x, self.tmp_kiwi_y = None, None
        self.tmp_bowl_x, self.tmp_bowl_y = None, None
        self.tmp_wood_x, self.tmp_wood_y = None, None
        # # 随机 木盘位置
        # wood_x_bios = (np.random.random()) * 0.02
        # # wood_x_bios = (np.random.random()) * 0.1
        # wood_y_bios = (np.random.random() - 1) * 0.05
        # # wood_y_bios = (np.random.random() - 1) * 0.1
        # self.mj_data.qpos[self.njq + 7 * 1 + 0] += wood_x_bios
        # self.mj_data.qpos[self.njq + 7 * 1 + 1] += wood_y_bios
        # self.mj_data.qpos[self.njq + 7 * 1 + 2] += 0.01
        # print("wood", self.mj_data.qpos[self.njq + 7 * 1 + 0], self.mj_data.qpos[self.njq + 7 * 1 + 1])
        #
        # # 随机 猕猴桃位置
        # kiwi_x_bios = (np.random.random() - 0.5) * 0.04
        # # kiwi_x_bios = (np.random.random() - 0.5) * 0.2
        # kiwi_y_bios = (np.random.random()) * 0.12
        # # kiwi_y_bios = (np.random.random()) * 0.1
        # self.mj_data.qpos[self.njq + 7 * 2 + 0] += kiwi_x_bios
        # self.mj_data.qpos[self.njq + 7 * 2 + 1] += kiwi_y_bios
        # print("kiwi", self.mj_data.qpos[self.njq + 7 * 2 + 0], self.mj_data.qpos[self.njq + 7 * 2 + 1])
        #
        # # 随机 碗位置
        # bowl_x_bios = (np.random.random()) * 0.02
        # # bowl_x_bios = (np.random.random()) * 0.1
        # bowl_y_bios = (np.random.random()) * 0.04
        # # bowl_y_bios = (np.random.random()) * 0.2
        # self.mj_data.qpos[self.njq + 7 * 0 + 0] += bowl_x_bios
        # self.mj_data.qpos[self.njq + 7 * 0 + 1] += bowl_y_bios
        # print("bowl", self.mj_data.qpos[self.njq + 7 * 0 + 0], self.mj_data.qpos[self.njq + 7 * 0 + 1])
        cnt = 0
        while not self.is_legal():
            cnt += 1
            if cnt > self.max_trials:
                raise RuntimeError("Fail to generate a proper scene")

        self.mj_data.qpos[self.njq + 7 * 1 + 0] = self.tmp_wood_x
        self.mj_data.qpos[self.njq + 7 * 1 + 1] = self.tmp_wood_y
        self.mj_data.qpos[self.njq + 7 * 1 + 2] = self.tmp_wood_z

        self.mj_data.qpos[self.njq + 7 * 2 + 0] = self.tmp_kiwi_x
        self.mj_data.qpos[self.njq + 7 * 2 + 1] = self.tmp_kiwi_y

        self.mj_data.qpos[self.njq + 7 * 0 + 0] = self.tmp_bowl_x
        self.mj_data.qpos[self.njq + 7 * 0 + 1] = self.tmp_bowl_y

    def generate_data(self):
        # 随机 木盘位置
        wood_x_bios = (np.random.random()) * 0.02
        # wood_x_bios = (np.random.random() - 1) * 0.1
        wood_y_bios = (np.random.random() - 1) * 0.05
        # wood_y_bios = (np.random.random() - 1) * 0.1
        self.tmp_wood_x = self.mj_data.qpos[self.njq + 7 * 1 + 0] + wood_x_bios
        self.tmp_wood_y = self.mj_data.qpos[self.njq + 7 * 1 + 1] + wood_y_bios
        self.tmp_wood_z = self.mj_data.qpos[self.njq + 7 * 1 + 2] + 0.01
        # print("wood", self.tmp_wood_x, self.tmp_wood_y, self.tmp_wood_z)

        # 随机 猕猴桃位置
        kiwi_x_bios = (np.random.random() - 0.5) * 0.04
        # kiwi_x_bios = (np.random.random() - 1.5) * 0.1
        kiwi_y_bios = (np.random.random()) * 0.12
        # kiwi_y_bios = (np.random.random()) * 0.12
        self.tmp_kiwi_x = self.mj_data.qpos[self.njq + 7 * 2 + 0] + kiwi_x_bios
        self.tmp_kiwi_y = self.mj_data.qpos[self.njq + 7 * 2 + 1] + kiwi_y_bios
        # print("kiwi", self.tmp_kiwi_x, self.tmp_kiwi_y)

        # 随机 碗位置
        bowl_x_bios = (np.random.random()) * 0.02
        # bowl_x_bios = (np.random.random()-0.5) * 0.1
        bowl_y_bios = (np.random.random()) * 0.04
        # bowl_y_bios = (np.random.random()) * 0.1
        self.tmp_bowl_x = self.mj_data.qpos[self.njq + 7 * 0 + 0] + bowl_x_bios
        self.tmp_bowl_y = self.mj_data.qpos[self.njq + 7 * 0 + 1] + bowl_y_bios
        # print("bowl", self.tmp_bowl_x, self.tmp_bowl_y)

    def is_legal(self):
        self.generate_data()

        # 确保生成的物体都在桌上
        objects = {"kiwi": (self.tmp_kiwi_x, self.tmp_kiwi_y), "bowl": (self.tmp_bowl_x, self.tmp_bowl_y), "wood": (self.tmp_wood_x, self.tmp_wood_y)}
        for obj, coordinates in objects.items():
            if ~(TABLE_WIDTH_LOWER_BOUND < coordinates[0] < TABLE_WIDTH_UPPER_BOUND and TABLE_LENGTH_LOWER_BOUND < coordinates[1] < TABLE_LENGTH_UPPER_BOUND):
                return False

        return True


    def check_success(self):
        print("The height of the bowl", self.mj_data.qpos[self.njq+7*0+2])
        print("The height of the Kiwi", self.mj_data.qpos[self.njq+7*2+2])
        if self.mj_data.qpos[self.njq+7*2+2] - self.mj_data.qpos[self.njq+7*0+2] > 1e-2:
            return False

        v1 = np.array([self.mj_data.qpos[self.njq+7*2+0], self.mj_data.qpos[self.njq+7*2+1]])
        v2 = np.array([self.mj_data.qpos[self.njq+7*0+0], self.mj_data.qpos[self.njq+7*0+1]])
        
        # 计算差异
        diff = v1 - v2
        
        # 计算平方和
        squared_diff = np.sum(diff**2)
        
        # 取平方根
        distance = np.sqrt(squared_diff) - 0.0195
        print("The Distance:\t", distance)
        if distance < 1e-2:
            return True

        return False
    
cfg = MMK2Cfg()
cfg.use_gaussian_renderer = False
cfg.init_key = "pick"
cfg.gs_model_dict["bowl_yellow"]     = "object/yellow_bowl.ply"
cfg.gs_model_dict["wood"]            = "object/wood.ply"
cfg.gs_model_dict["kiwi"]          = "object/kiwi.ply"
# cfg.gs_model_dict["background"]      = "scene/Lab3/environment.ply"
cfg.gs_model_dict["background"]      = "scene/tsimf_library_0/environment.ply"


cfg.mjcf_file_path = "mjcf/tasks_mmk2/pick_kiwi.xml"
cfg.obj_list    = ["bowl_yellow", "wood", "kiwi"]
cfg.sync     = False
cfg.headless = False
cfg.render_set  = {
    "fps"    : 30,
    "width"  : 640,
    "height" : 480
}
cfg.obs_rgb_cam_id = [0, 1, 2]
cfg.save_mjb_and_task_config = False

if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True, linewidth=500)

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_idx", type=int, default=0, help="data index")
    parser.add_argument("--data_set_size", type=int, default=1, help="data set size")
    parser.add_argument("--auto", action="store_true", help="auto run")
    args = parser.parse_args()

    data_idx, data_set_size = args.data_idx, args.data_idx + args.data_set_size
    if args.auto:
        cfg.headless = True
        cfg.sync = False

    save_dir = os.path.join(DISCOVERSE_ROOT_DIR, "data/mmk2_pick_kiwi")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    sim_node = SimNode(cfg)
    sim_node.teleop = None
    if hasattr(cfg, "save_mjb_and_task_config") and cfg.save_mjb_and_task_config:
        mujoco.mj_saveModel(sim_node.mj_model, os.path.join(save_dir, os.path.basename(cfg.mjcf_file_path).replace(".xml", ".mjb")))
        shutil.copyfile(os.path.abspath(__file__), os.path.join(save_dir, os.path.basename(__file__)))

    stm = SimpleStateMachine()
    stm.max_state_cnt = 19
    max_time = 60.0 #s

    action = np.zeros_like(sim_node.target_control)
    process_list = []

    move_speed = 1.
    obs = sim_node.reset()
    while sim_node.running:
        if sim_node.reset_sig:
            sim_node.reset_sig = False
            stm.reset()
            action[:] = sim_node.target_control[:]
            act_lst, obs_lst = [], []

        try:
            if stm.trigger():
                if stm.state_idx == 0: # 降高度
                    sim_node.tctr_head[1] = 1
                    sim_node.tctr_slide[0] = 0.2
                elif stm.state_idx == 1: # 伸到碗前
                    tmat_bowl = get_body_tmat(sim_node.mj_data, "bowl_yellow")
                    target_posi = tmat_bowl[:3, 3] + 0.1 * tmat_bowl[:3, 1] + 0.1 * tmat_bowl[:3, 2]
                    sim_node.lft_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.lft_arm_target_pose, sim_node.arm_action, "l", sim_node.sensor_lft_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                    sim_node.tctr_lft_gripper[:] = 1
                elif stm.state_idx == 2: # 伸到碗壁
                    tmat_bowl = get_body_tmat(sim_node.mj_data, "bowl_yellow")
                    target_posi = tmat_bowl[:3, 3] + 0.046 * tmat_bowl[:3, 1] + 0.05 * tmat_bowl[:3, 2]
                    sim_node.lft_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.lft_arm_target_pose, sim_node.arm_action, "l", sim_node.sensor_lft_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                elif stm.state_idx == 3: # 抓住碗壁
                    sim_node.tctr_lft_gripper[:] = 0.0
                elif stm.state_idx == 4: # 提起来碗
                    sim_node.tctr_slide[0] = 0.0
                elif stm.state_idx == 5: # 把碗放到托盘上空
                    tmat_plate = get_body_tmat(sim_node.mj_data, "wood")
                    target_posi = tmat_plate[:3, 3] + np.array([0.0, 0.045, 0.25])
                    sim_node.lft_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.lft_arm_target_pose, sim_node.arm_action, "l", sim_node.sensor_lft_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                elif stm.state_idx == 6: # 下降高度
                    sim_node.tctr_slide[0] = 0.18
                elif stm.state_idx == 7: # 松开碗壁 放下碗
                    sim_node.tctr_lft_gripper[:] = 1
                elif stm.state_idx == 8: # 上升高度
                    sim_node.tctr_slide[0] = 0.1
                elif stm.state_idx == 9: # 移开手臂
                    tmat_plate = get_body_tmat(sim_node.mj_data, "wood")
                    target_posi = tmat_plate[:3, 3] + np.array([0.2, 0.03, 0.25])
                    sim_node.lft_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.lft_arm_target_pose, sim_node.arm_action, "l", sim_node.sensor_lft_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                elif stm.state_idx == 10: # 伸到猕猴桃上
                    tmat_kiwi = get_body_tmat(sim_node.mj_data, "kiwi")
                    target_posi = tmat_kiwi[:3, 3] + 0.1 * tmat_kiwi[:3, 1] + 0.2 * tmat_kiwi[:3, 2]
                    sim_node.rgt_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.rgt_arm_target_pose, sim_node.arm_action, "r", sim_node.sensor_rgt_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                    sim_node.tctr_rgt_gripper[:] = 1
                elif stm.state_idx == 11: # 伸到猕猴桃前
                    tmat_kiwi = get_body_tmat(sim_node.mj_data, "kiwi")
                    target_posi = tmat_kiwi[:3, 3] + 0.045 * tmat_kiwi[:3, 0] + 0.132 * tmat_kiwi[:3, 2]
                    sim_node.rgt_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.rgt_arm_target_pose, sim_node.arm_action, "r", sim_node.sensor_rgt_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                    sim_node.tctr_rgt_gripper[:] = 1
                elif stm.state_idx == 12: # 降高度
                    # sim_node.tctr_head[1] = 0.6
                    sim_node.tctr_slide[0] = 0.2
                elif stm.state_idx == 13: # 抓住猕猴桃
                    sim_node.tctr_rgt_gripper[:] = 0.5
                elif stm.state_idx == 14: # 提起猕猴桃
                    sim_node.tctr_slide[0] = 0.0
                elif stm.state_idx == 15: # 移动到碗上空
                    tmat_bowl = get_body_tmat(sim_node.mj_data, "bowl_yellow")
                    target_posi = tmat_bowl[:3, 3] + 0.2 * tmat_bowl[:3, 2] + 0.015 * tmat_bowl[:3, 0]
                    sim_node.rgt_arm_target_pose[:] = sim_node.get_tmat_wrt_mmk2base(target_posi)
                    sim_node.setArmEndTarget(sim_node.rgt_arm_target_pose, sim_node.arm_action, "r", sim_node.sensor_rgt_arm_qpos, Rotation.from_euler('zyx', [ 0., -0.0551, 0.]).as_matrix())
                elif stm.state_idx == 16: # 降高度
                    # sim_node.tctr_head[1] = 0.8
                    sim_node.tctr_slide[0] = 0.12
                elif stm.state_idx == 17: # 放开猕猴桃
                    sim_node.tctr_rgt_gripper[:] = 1.0
                elif stm.state_idx == 18: # 升高度
                    # sim_node.tctr_head[1] = 0.8
                    sim_node.tctr_slide[0] = 0.05
                
                dif = np.abs(action - sim_node.target_control)
                sim_node.joint_move_ratio = dif / (np.max(dif) + 1e-6)
                sim_node.joint_move_ratio[2] *= 0.25

            elif sim_node.mj_data.time > max_time:
                raise ValueError("Time out")

            else:
                stm.update()

            if sim_node.checkActionDone():
                stm.next()

        except ValueError as ve:
            print(ve)
            sim_node.reset()

        for i in range(2, sim_node.njctrl):
            action[i] = step_func(action[i], sim_node.target_control[i], move_speed * sim_node.joint_move_ratio[i] * sim_node.delta_t)
        yaw = Rotation.from_quat(np.array(obs["base_orientation"])[[1,2,3,0]]).as_euler("xyz")[2] + np.pi / 2
        action[1] = -10 * yaw + sim_node.target_control[1]
        # action[1] = sim_node.target_control[1]

        obs, _, _, _, _ = sim_node.step(action)
        
        if len(obs_lst) < sim_node.mj_data.time * cfg.render_set["fps"]:
            act_lst.append(action.tolist().copy())
            obs_lst.append(obs)

        if stm.state_idx >= stm.max_state_cnt:
            if sim_node.check_success():
                save_path = os.path.join(save_dir, "{:03d}".format(data_idx))
                process = mp.Process(target=recoder_mmk2, args=(save_path, act_lst, obs_lst, cfg))
                process.start()
                process_list.append(process)

                data_idx += 1
                print("\r{:4}/{:4} ".format(data_idx, data_set_size), end="")
                if data_idx >= data_set_size:
                    break
            else:
                print(f"{data_idx} Failed")

            obs = sim_node.reset()

    for p in process_list:
        p.join()
