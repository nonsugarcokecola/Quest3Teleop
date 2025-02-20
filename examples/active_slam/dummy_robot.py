import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from discoverse.envs import SimulatorBase
from discoverse.utils.base_config import BaseConfig

class DummyRobotConfig(BaseConfig):
    y_up = False
    robot_name = "dummy_robot"

class DummyRobot(SimulatorBase):
    Tmat_Zup2Yup = np.array([
        [1, 0,  0, 0],
        [0, 0, -1, 0],
        [0, 1,  0, 0],
        [0, 0,  0, 1]
    ])
    move_step_ratio = 1.0
    pitch_joint_id = 0
    def __init__(self, config: DummyRobotConfig):
        super().__init__(config)

    def updateControl(self, action):
        move_step = action * self.config.timestep
        self.base_move(*tuple(move_step))

    def get_base_pose(self):
        return self.mj_model.body(self.config.robot_name).pos.copy(), self.mj_model.body(self.config.robot_name).quat.copy()

    def getObservation(self):
        rgb_cam_pose_lst = [self.getCameraPose(id) for id in self.config.obs_rgb_cam_id]
        depth_cam_pose_lst = [self.getCameraPose(id) for id in self.config.obs_depth_cam_id]
        if self.config.y_up:
            for pose_lst in [rgb_cam_pose_lst, depth_cam_pose_lst]:
                for i, (xyz, quat_wxyz) in enumerate(pose_lst):
                    Tmat = np.eye(4)
                    Tmat[:3, :3] = Rotation.from_quat(quat_wxyz[[1,2,3,0]]).as_matrix()
                    Tmat[:3, 3] = xyz[:]
                    new_Tmat = np.linalg.inv(self.Tmat_Zup2Yup) @ Tmat
                    pose_lst[i] = (new_Tmat[:3, 3], Rotation.from_matrix(new_Tmat[:3, :3]).as_quat()[[3,0,1,2]])
        self.obs = {
            "rgb_cam_posi"   : rgb_cam_pose_lst,
            "depth_cam_posi" : depth_cam_pose_lst,
            "rgb_img"        : self.img_rgb_obs_s,
            "depth_img"      : self.img_depth_obs_s,
        }
        return self.obs

    def getPrivilegedObservation(self):
        return self.obs    

    def checkTerminated(self):
        return False
    
    def getReward(self):
        return None

    def cv2MouseCallback(self):
        global action
        if self.mouseParam[3] & cv2.EVENT_FLAG_SHIFTKEY:
            self.move_step_ratio = 5.0
        else:
            self.move_step_ratio = 1.0

        if self.cam_id == -1:
            super().cv2MouseCallback()
        else:
            event = self.mouseParam[0]
            x = self.mouseParam[1]
            y = self.mouseParam[2]
            flags = self.mouseParam[3]
            if flags == cv2.EVENT_FLAG_LBUTTON and event == cv2.EVENT_MOUSEMOVE:
                self.camera_pose_changed = True
                height = self.config.render_set["height"]
                dx = float(x) - self.mouse_last_x
                dy = float(y) - self.mouse_last_y
                self.base_move(0.0, 0.0, -dx/height, 0.0)
                self.move_camera_pitch(dy/height)
            self.mouse_last_x = float(x)
            self.mouse_last_y = float(y)

    def move_camera_pitch(self, d_pitch):
        self.mj_data.qpos[self.pitch_joint_id] += d_pitch

    def base_move(self, dx_local, dy_local, angular_z, pitch_local):
        posi_, quat_wxyz = self.get_base_pose()
        yaw = Rotation.from_quat(quat_wxyz[[1,2,3,0]]).as_euler("zyx")[0]
        yaw += angular_z
        if yaw > np.pi:
            yaw -= 2. * np.pi
        elif yaw < -np.pi:
            yaw += 2. * np.pi
        self.mj_model.body(self.config.robot_name).quat[:] = Rotation.from_euler("zyx", [yaw, 0, 0]).as_quat()[[3,0,1,2]]

        base_posi = self.mj_model.body(self.config.robot_name).pos
        base_posi[0] += dx_local * np.cos(yaw) - dy_local * np.sin(yaw)
        base_posi[1] += dx_local * np.sin(yaw) + dy_local * np.cos(yaw)
        self.mj_data.qpos[0] += pitch_local
        self.mj_data.qpos[0] = np.clip(self.mj_data.qpos[0], -np.pi/2., np.pi/2.)

    def teleopProcess(self):
        pass

    def cv2WindowKeyPressCallback(self, key):
        ret = super().cv2WindowKeyPressCallback(key)
        step = 1.0 / float(self.config.render_set["fps"]) * 5. * self.move_step_ratio
        dx = 0.0
        dy = 0.0
        dz = 0.0
        dpitch = 0.0
        if key == 82:
            dx = step
        elif key == 84:
            dx = -step
        if key == 81:
            dy = step
        elif key == 83:
            dy = -step
        elif key == ord("q"):
            dpitch = 0.05
        elif key == ord("w"):
            dpitch = -0.05
        elif key == ord("z"):
            self.mj_model.body(self.config.robot_name).pos[2] += 0.02
        elif key == ord("x"):
            self.mj_model.body(self.config.robot_name).pos[2] -= 0.02
        self.base_move(dx, dy, dz, dpitch)
        return ret

    def printHelp(self):
        super().printHelp()
        print("-------------------------------------")
        print("dummy robot control:")
        print("arrow up/arrow down    : move forward/backward")
        print("arrow left/arrow right : move left/right")
        print("q/w : pitch up/down")
        print("z/x : height up/down")
        print("left mouse drag : camera move yaw and pitch")
        print("press shift key to move faster")

if __name__ == "__main__":
    cfg = DummyRobotConfig()
    cfg.y_up = False
    
    camera_height = 1.0 #m

    dummy_robot_cam_id = 0
    cfg.obs_rgb_cam_id   = [dummy_robot_cam_id]
    cfg.obs_depth_cam_id = [dummy_robot_cam_id]

    cfg.render_set["fps"] = 60
    cfg.render_set["width"] = 1920
    cfg.render_set["height"] = 1080
    cfg.timestep = 1./cfg.render_set["fps"]
    cfg.decimation = 1
    cfg.mjcf_file_path = "mjcf/dummy_robot.xml"

    cfg.use_gaussian_renderer = True
    cfg.gs_model_dict["background"] = "scene/Air11F/air_11f.ply"

    robot = DummyRobot(cfg)
    robot.cam_id = dummy_robot_cam_id

    robot.mj_model.body("dummy_robot").pos[2] = camera_height

    action = np.zeros(4)
    # if z_up:
        # action[0] : lineal_velocity_x  local m    不论yup还是zup，始终为朝前为正方向
        # action[1] : lineal_velocity_y  local m    不论yup还是zup，始终为朝左为正方向
        # action[2] : angular_velocity_z rad        不论yup还是zup，始终为从上向下看逆时针旋转为正方向
        # action[3] : camera_pitch       rad        不论yup还是zup，始终为镜头俯仰
    # elif y_up:
        # action[0] : lineal_velocity_x   local m
        # action[1] : lineal_velocity_-z  local m
        # action[2] : angular_velocity_y  rad
        # action[3] : camera_pitch        rad 

    obs = robot.reset()
    rgb_cam_posi = obs["rgb_cam_posi"]
    depth_cam_posi = obs["depth_cam_posi"]
    rgb_img_0 = obs["rgb_img"][0]
    depth_img_0 = obs["depth_img"][0]

    print("rgb_cam_posi    = ", rgb_cam_posi)
    # [[posi_x, posi_y, posi_z], [quat_w, quat_x, quat_y, quat_z]]
    # [(array([0., 0., 1.]), array([ 0.49999816,  0.50000184, -0.5       , -0.5       ]))]

    print("depth_cam_posi  = ", depth_cam_posi)
    # [[posi_x, posi_y, posi_z], [quat_w, quat_x, quat_y, quat_z]]
    # [(array([0., 0., 1.]), array([ 0.49999816,  0.50000184, -0.5       , -0.5       ]))]

    print("rgb_img.shape   = ", rgb_img_0.shape  , "rgb_img.dtype    = ", rgb_img_0.dtype)
    # rgb_img.shape   =  (1080, 1920, 3) rgb_img.dtype    =  uint8

    print("depth_img.shape = ", depth_img_0.shape, "depth_img.dtype  = ", depth_img_0.dtype)
    # depth_img.shape =  (1080, 1920, 1) depth_img.dtype  =  float32

    robot.printHelp()

    while robot.running:
        obs, _, _, _, _ = robot.step(action)