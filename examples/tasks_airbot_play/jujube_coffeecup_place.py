import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import os
import shutil
import argparse
import multiprocessing as mp

from discoverse.airbot_play import AirbotPlayFIK
from discoverse import DISCOVERSE_ROOT_DIR, DISCOVERSE_ASSERT_DIR
from discoverse.envs.airbot_play_base import AirbotPlayCfg
from discoverse.utils import get_body_tmat, get_site_tmat, step_func, SimpleStateMachine
from discoverse.task_base import AirbotPlayTaskBase, recoder_airbot_play


class SimNode(AirbotPlayTaskBase):
    random_level = 3
    def __init__(self, config: AirbotPlayCfg):
        super().__init__(config)
        self.camera_0_pose = (self.mj_model.camera("eye_side").pos.copy(), self.mj_model.camera("eye_side").quat.copy())

    def domain_randomization(self):
        if self.random_level == 0: #200
            # 随机 枣子位置
            self.mj_data.qpos[self.nj+1+0] += 2.*(np.random.random() - 0.5) * 0.05
            self.mj_data.qpos[self.nj+1+1] += 2.*(np.random.random() - 0.5) * 0.025
            # 随机 杯子位置
            self.mj_data.qpos[self.nj+7+1+0] += 2.*(np.random.random() - 0.5) * 0.05
            self.mj_data.qpos[self.nj+7+1+1] += 2.*(np.random.random() - 0.5) * 0.025

        elif self.random_level == 1: #400
            # 随机 枣子位置
            self.mj_data.qpos[self.nj+1+0] += 2.*(np.random.random() - 0.5) * 0.05
            self.mj_data.qpos[self.nj+1+1] += 2.*(np.random.random() - 0.5) * 0.025
            # 随机 杯子位置
            self.mj_data.qpos[self.nj+7+1+0] += 2.*(np.random.random() - 0.5) * 0.05
            self.mj_data.qpos[self.nj+7+1+1] += 2.*(np.random.random() - 0.5) * 0.025
            # 随机 盘子位置
            self.mj_data.qpos[self.nj+7*2+1+0] += 2.*(np.random.random() - 0.5) * 0.0625
            self.mj_data.qpos[self.nj+7*2+1+1] += 2.*(np.random.random() - 0.5) * 0.02

        elif self.random_level == 2: #800
            # 随机 枣子位置
            self.mj_data.qpos[self.nj+1+0] += 2.*(np.random.random() - 0.5) * 0.05
            self.mj_data.qpos[self.nj+1+1] += 2.*(np.random.random() - 0.5) * 0.025
            # 随机 杯子位置
            self.mj_data.qpos[self.nj+7+1+0] += 2.*(np.random.random() - 0.5) * 0.05
            self.mj_data.qpos[self.nj+7+1+1] += 2.*(np.random.random() - 0.5) * 0.025

            wood_base_bias = 2.*(np.random.random(2) - 0.5) * 0.035
            # 随机 盘子位置
            self.mj_data.qpos[self.nj+7*2+1+0] += 2.*(np.random.random() - 0.5) * 0.0625 + wood_base_bias[0]
            self.mj_data.qpos[self.nj+7*2+1+1] += 2.*(np.random.random() - 0.5) * 0.02   + wood_base_bias[1]
            # 随机 木板位置
            self.mj_data.qpos[self.nj+7*3+1+0] += wood_base_bias[0]
            self.mj_data.qpos[self.nj+7*3+1+1] += 2.*(np.random.random() - 0.5) * 0.02 + wood_base_bias[1]

        elif self.random_level == 3: #1000
            # 随机 枣子位置
            self.mj_data.qpos[self.nj+1+0] += 2.*(np.random.random() - 0.5) * 0.1
            self.mj_data.qpos[self.nj+1+1] += 2.*(np.random.random() - 0.5) * 0.05
            # 随机 杯子位置
            self.mj_data.qpos[self.nj+7+1+0] += 2.*(np.random.random() - 0.5) * 0.1
            self.mj_data.qpos[self.nj+7+1+1] += 2.*(np.random.random() - 0.5) * 0.05
            wood_base_bias = 2.*(np.random.random(2) - 0.5) * 0.05
            # 随机 盘子位置
            self.mj_data.qpos[self.nj+7*2+1+0] += 2.*(np.random.random() - 0.5) * 0.0625 + wood_base_bias[0]
            self.mj_data.qpos[self.nj+7*2+1+1] += 2.*(np.random.random() - 0.5) * 0.02   + wood_base_bias[1]
            # 随机 木板位置
            self.mj_data.qpos[self.nj+7*3+1+0] += wood_base_bias[0]
            self.mj_data.qpos[self.nj+7*3+1+1] += 2.*(np.random.random() - 0.5) * 0.02 + wood_base_bias[1]


    def check_success(self):
        tmat_cup = get_body_tmat(self.mj_data, "coffeecup_white")
        tmat_plate = get_body_tmat(self.mj_data, "plate_white")
        return (abs(tmat_cup[2, 2]) > 0.99) and np.hypot(tmat_plate[0, 3] - tmat_cup[0, 3], tmat_plate[1, 3] - tmat_cup[1, 3]) < 0.02

cfg = AirbotPlayCfg()
cfg.use_gaussian_renderer = False
cfg.init_key = "ready"
cfg.gs_model_dict["background"]      = "scene/lab3/point_cloud.ply"
cfg.gs_model_dict["drawer_1"]        = "hinge/drawer_1.ply"
cfg.gs_model_dict["drawer_2"]        = "hinge/drawer_2.ply"
cfg.gs_model_dict["plate_white"]     = "object/plate_white.ply"
cfg.gs_model_dict["coffeecup_white"] = "object/teacup.ply"
cfg.gs_model_dict["wood"]            = "object/wood.ply"
cfg.gs_model_dict["jujube"]          = "object/jujube.ply"

cfg.mjcf_file_path = "mjcf/tasks_airbot_play/jujube_coffeecup_place.xml"
cfg.obj_list     = ["drawer_1", "drawer_2", "jujube", "plate_white", "coffeecup_white", "wood"]
cfg.timestep     = 1/240
cfg.decimation   = 4
cfg.sync         = True
cfg.headless     = False
cfg.render_set   = {
    "fps"    : 20,
    "width"  : 640, #1920, #640,
    "height" : 480, #1080  #480
}
cfg.obs_rgb_cam_id = [0, 1]
cfg.save_mjb_and_task_config = True

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

    save_dir = os.path.join(DISCOVERSE_ROOT_DIR, "data/jujube_coffeecup_place")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    sim_node = SimNode(cfg)
    if hasattr(cfg, "save_mjb_and_task_config") and cfg.save_mjb_and_task_config and data_idx == 0:
        mujoco.mj_saveModel(sim_node.mj_model, os.path.join(save_dir, os.path.basename(cfg.mjcf_file_path).replace(".xml", ".mjb")))
        shutil.copyfile(os.path.abspath(__file__), os.path.join(save_dir, os.path.basename(__file__)))

    sim_node.free_camera.distance = 0.5
    sim_node.mj_model.vis.global_.fovy = 72.02033809218685
    sim_node.cam_id = 0
    # sim_node.gs_renderer.set_obj_pose("table_cloth", [0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0])
    arm_fik = AirbotPlayFIK(os.path.join(DISCOVERSE_ASSERT_DIR, "urdf/airbot_play_v3_gripper_fixed.urdf"))

    trmat = Rotation.from_euler("xyz", [0, np.pi*0.4, 0], degrees=False).as_matrix()
    tmat_armbase_2_world = np.linalg.inv(get_body_tmat(sim_node.mj_data, "arm_base"))

    stm = SimpleStateMachine()
    stm.max_state_cnt = 19
    max_time = 20.0 #s

    action = np.zeros(7)
    process_list = []

    move_speed = 0.75
    sim_node.reset()
    while sim_node.running:
        if sim_node.reset_sig:
            sim_node.reset_sig = False
            stm.reset()
            action[:] = sim_node.target_control[:]
            act_lst, obs_lst = [], []

        try:
            if stm.trigger():
                if stm.state_idx == 0: # 伸到枣上方
                    tmat_jujube = get_body_tmat(sim_node.mj_data, "jujube")
                    tmat_jujube[:3, 3] = tmat_jujube[:3, 3] + 0.1 * tmat_jujube[:3, 2]
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_jujube
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                    sim_node.target_control[6] = 1
                elif stm.state_idx == 1: # 伸到枣
                    tmat_jujube = get_body_tmat(sim_node.mj_data, "jujube")
                    tmat_jujube[:3, 3] = tmat_jujube[:3, 3] + 0.027 * tmat_jujube[:3, 2]
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_jujube
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 2: # 抓住枣
                    sim_node.target_control[6] = 0
                elif stm.state_idx == 3: # 抓稳枣
                    sim_node.delay_cnt = int(0.35/sim_node.delta_t)
                elif stm.state_idx == 4: # 提起来枣
                    tmat_tgt_local[2,3] += 0.15
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 5: # 把枣放到杯子上空
                    tmat_plate = get_body_tmat(sim_node.mj_data, "coffeecup_white")
                    tmat_plate[:3,3] = tmat_plate[:3, 3] + np.array([0.0, 0.0, 0.13])
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_plate
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 6: # 降低高度 把枣放到杯子上
                    tmat_tgt_local[2,3] -= 0.01
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 7: # 松开枣
                    sim_node.target_control[6] = 1
                elif stm.state_idx == 8: # 抬升高度
                    tmat_tgt_local[2,3] += 0.05
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 9: # 伸到杯子前
                    tmat_coffee = get_body_tmat(sim_node.mj_data, "coffeecup_white")
                    tmat_coffee[:3, 3] = tmat_coffee[:3, 3] + 0.1 * tmat_coffee[:3, 1] + 0.1 * tmat_coffee[:3, 2]
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_coffee
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                    sim_node.target_control[6] = 1
                elif stm.state_idx == 10: # 伸到杯把
                    tmat_coffee = get_body_tmat(sim_node.mj_data, "coffeecup_white")
                    tmat_coffee[:3, 3] = tmat_coffee[:3, 3] + 0.06 * tmat_coffee[:3, 1] + 0.05 * tmat_coffee[:3, 2]
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_coffee
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 11: # 抓住杯把
                    sim_node.target_control[6] = 0
                elif stm.state_idx == 12: # 抓住杯把
                    sim_node.delay_cnt = int(0.5/sim_node.delta_t)
                elif stm.state_idx == 13: # 提起来杯子
                    tmat_tgt_local[2,3] += 0.15
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 14: # 把杯子放到盘子上空
                    tmat_plate = get_body_tmat(sim_node.mj_data, "plate_white")
                    tmat_plate[:3,3] = tmat_plate[:3, 3] + np.array([0.06, 0.0, 0.13])
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_plate
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 15: # 把杯子放到盘子上空
                    tmat_plate = get_body_tmat(sim_node.mj_data, "plate_white")
                    tmat_plate[:3,3] = tmat_plate[:3, 3] + np.array([0.06, 0.0, 0.08])
                    tmat_tgt_local = tmat_armbase_2_world @ tmat_plate
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 16: # 降低高度 把杯子放到盘子上
                    tmat_tgt_local[2,3] -= 0.02
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])
                elif stm.state_idx == 17: # 松开杯把 放下杯子
                    sim_node.target_control[6] = 1
                elif stm.state_idx == 18: # 抬升高度
                    tmat_tgt_local[2,3] += 0.05
                    sim_node.target_control[:6] = arm_fik.properIK(tmat_tgt_local[:3,3], trmat, sim_node.mj_data.qpos[:6])

                dif = np.abs(action - sim_node.target_control)
                sim_node.joint_move_ratio = dif / (np.max(dif) + 1e-6)

            elif sim_node.mj_data.time > max_time:
                raise ValueError("Time out")

            else:
                stm.update()

            if sim_node.checkActionDone():
                stm.next()

        except ValueError as ve:
            # traceback.print_exc()
            sim_node.reset()

        for i in range(sim_node.nj-1):
            action[i] = step_func(action[i], sim_node.target_control[i], move_speed * sim_node.joint_move_ratio[i] * sim_node.delta_t)
        action[6] = sim_node.target_control[6]

        obs, _, _, _, _ = sim_node.step(action)

        if len(obs_lst) < sim_node.mj_data.time * cfg.render_set["fps"]:
            act_lst.append(action.tolist().copy())
            obs_lst.append(obs)

        if stm.state_idx >= stm.max_state_cnt:
            if sim_node.check_success():
                save_path = os.path.join(save_dir, "{:03d}".format(data_idx))
                process = mp.Process(target=recoder_airbot_play, args=(save_path, act_lst, obs_lst, cfg))
                process.start()
                process_list.append(process)

                data_idx += 1
                print("\r{:4}/{:4} ".format(data_idx, data_set_size), end="")
                if data_idx >= data_set_size:
                    break
            else:
                print(f"{data_idx} Failed")

            sim_node.reset()

    for p in process_list:
        p.join()
