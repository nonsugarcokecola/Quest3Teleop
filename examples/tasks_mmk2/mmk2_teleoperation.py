import numpy as np

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from discoverse import DISCOVERSE_ROOT_DIR
from discoverse.examples.tasks_mmk2.subscriber.mmk2_platform_subscriber import MMK2PlatformSubscriber
from discoverse.examples.tasks_mmk2.subscriber.mmk2_gripper_subscriber import MMk2GripperSubscriber
from discoverse.examples.tasks_mmk2.subscriber.mmk2_arm_subscriber import MMK2ARMSubscriber

from discoverse.envs.mmk2_base import MMK2Cfg
from mmk2_types.types import MMK2Components, ImageTypes, ControllerTypes
from mmk2_types.grpc_msgs import (
    JointState,
    TrajectoryParams,
    MoveServoParams,
    GoalStatus,
    BaseControlParams,
    BuildMapParams,
    Pose3D,
    Twist3D,
    BaseChargeStationParams,
    ArrayStamped,
)
from airbot_py.airbot_mmk2 import AirbotMMK2
import logging

from publisher.image_publisher import MMK2CameraPublisher
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

mmk2 = AirbotMMK2(ip="192.168.11.200")

default_joint_action = {
    MMK2Components.LEFT_ARM: JointState(position=[0.0, 0.0, 0.324, 0.0, 0.724, 0.0]),
    MMK2Components.RIGHT_ARM: JointState(position=[0.0, 0.0, 0.324, 0.0, -0.724, 0.0]),
    MMK2Components.LEFT_ARM_EEF: JointState(position=[0.0]),
    MMK2Components.RIGHT_ARM_EEF: JointState(position=[0.0]),
    MMK2Components.HEAD: JointState(position=[0.0, 0.18]),
    MMK2Components.SPINE: JointState(position=[0.0]),
}

mmk2.enable_resources(
    {
        MMK2Components.HEAD_CAMERA: {  # realsense case
            "camera_type": "REALSENSE",
            "rgb_camera.color_profile": "640,480,30",
            "enable_depth": "false",
        },
        # MMK2Components.LEFT_CAMERA: { # USB camera case
        #     "camera_type": "USB",
        #     "video_device": "/dev/left_camera",
        #     "image_width": "640",
        #     "image_height": "480",
        #     "framerate": "25",
        # },
        # MMK2Components.RIGHT_CAMERA: { # USB camera case
        #     "camera_type": "USB",
        #     "video_device": "/dev/right_camera",
        #     "image_width": "640",
        #     "image_height": "480",
        #     "framerate": "25",
        # },
    }
)

cam_mapping = {
    "head_camera": "mid_cam"
}

image_goal = {
    MMK2Components.HEAD_CAMERA: ImageTypes.RGB,
    # MMK2Components.LEFT_CAMERA: ImageTypes.RGB,
    # MMK2Components.RIGHT_CAMERA: ImageTypes.RGB,
}
mmk2.enable_stream(mmk2.get_image, image_goal)

# save_dir = os.path.join(DISCOVERSE_ROOT_DIR, "data/mmk2_pick_kiwi")
# if not os.path.exists(save_dir):
#     os.makedirs(save_dir)

def spin_node_in_thread(executor):
    executor.spin()

if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True, linewidth=500)

    rclpy.init()
    cam_publishers_node = MMK2CameraPublisher(mmk2)
    platform_subscriber_node = MMK2PlatformSubscriber(mmk2, default_joint_action)
    gripper_subscriber_node = MMk2GripperSubscriber(mmk2, default_joint_action)
    arm_subscriber_node = MMK2ARMSubscriber(mmk2, default_joint_action)

    executor = MultiThreadedExecutor()
    executor.add_node(cam_publishers_node)
    executor.add_node(platform_subscriber_node)
    executor.add_node(gripper_subscriber_node)
    executor.add_node(arm_subscriber_node)

    spin_thread = threading.Thread(target=spin_node_in_thread, args=(executor,))
    spin_thread.start()
    while True:
        comp_images = mmk2.get_image(image_goal)
        for comp, images in comp_images.items():
            if images.color.shape[0] == 1:
                print(f"{comp} got no image")
                break
            try:
                cam_publishers_node.update_image(cam_mapping[comp.value], images.color)
            except Exception as e:
                raise Exception(f"Invalid camera name {comp}: {e}")

    cam_publishers_node.destroy_node()
    platform_subscriber_node.destroy_node()
    gripper_subscriber_node.destroy_node()
    arm_subscriber_node.destroy_node()

    spin_thread.join()
    rclpy.shutdown()