import mujoco
import numpy as np

import os
import shutil
import argparse
import multiprocessing as mp

from discoverse.mmk2 import MMK2FIK
from discoverse import DISCOVERSE_ROOT_DIR, DISCOVERSE_ASSERT_DIR
from discoverse.envs.mmk2_base import MMK2Cfg
from discoverse.utils import get_body_tmat, get_site_tmat, step_func, SimpleStateMachine
from discoverse.task_base import MMK2TaskBase, recoder_mmk2

import traceback

class MMK2TASK(MMK2TaskBase):
    def domain_randomization(self): 
        # 随机生成杯子位置
        self.mj_data.qpos[self.nj+1+0] += 2.*(np.random.random() - 0.5) * 0.2
        self.mj_data.qpos[self.nj+1+1] += 2.*(np.random.random() - 0.5) * 0.06

        # 随机生成盘子位置
        self.mj_data.qpos[self.nj+7*1+1+0] += 2.*(np.random.random() - 0.5) * 0.0625
    
    def check_success(self):
        tmat_cup = get_body_tmat(self.mj_data, "coffeecup_white")
        tmat_plate = get_body_tmat(self.mj_data, "plate_white")
        return (abs(tmat_cup[2, 2]) > 0.99) and np.hypot(tmat_plate[0, 3] - tmat_cup[0, 3], tmat_plate[1, 3] - tmat_cup[1, 3]) < 0.02
    
cfg = MMK2Cfg()
cfg.use_gaussian_renderer = True
cfg.init_key = "ready"
cfg.gs_model_dict["background"]      = "scene/tsimf_library_0/point_cloud_for_mmk2.ply"
cfg.gs_model_dict["coffeecup_white"] = "object/teacup.ply"
cfg.gs_model_dict["plate_white"]     = "object/plate_white.ply"

cfg.mjcf_file_path = "mjcf/tasks_mmk2/cup_box_place.xml"
cfg.obj_list    = ["coffeecup_white", "plate_white"]

cfg.sync     = False
cfg.headless = False
cfg.render_set  = {
    "fps"    : 30,
    "width"  : 1920,
    "height" : 1080
}
cfg.obs_rgb_cam_id = None
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
    
    save_dir = os.path.join(DISCOVERSE_ROOT_DIR, "data/mmk2_plate_cooffecup")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    sim_node = MMK2TASK(cfg)
    sim_node.teleop = None
    if hasattr(cfg, "save_mjb_and_task_config") and cfg.save_mjb_and_task_config:
        mujoco.mj_saveModel(sim_node.mj_model, os.path.join(save_dir, os.path.basename(cfg.mjcf_file_path).replace(".xml", ".mjb")))
        shutil.copyfile(os.path.abspath(__file__), os.path.join(save_dir, os.path.basename(__file__)))
        
    stm= SimpleStateMachine()
    stm.max_state_cnt = 5
    max_time = 20.0 #s

    action = np.zeros_like(sim_node.target_control)
    process_list = []

    pick_lip_arm = "l"
    move_speed = 1.
    obs = sim_node.reset()
    while sim_node.running:
        if sim_node.reset_sig:
            sim_node.reset_sig = False
            sim_node.reset()
            action[:] = sim_node.target_control[:]
            act_lst, obs_lst = [], []

        try:
            pass
        except ValueError as ve:
            sim_node.reset()
            