import logging
import time

import numpy as np
from mmk2_types.types import MMK2Components
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

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def control_traj_servo_separate(mmk2, trgt_joint_action):
    freq = 20
    time_sec = 5
    action_ref = trgt_joint_action.copy()
    if (
        mmk2.set_goal(
            {MMK2Components.SPINE: action_ref.pop(MMK2Components.SPINE)},
            TrajectoryParams(),
        ).value
        == GoalStatus.Status.SUCCESS
    ):
        for _ in range(freq * time_sec):
            if (
                mmk2.set_goal(action_ref, MoveServoParams()).value
                != GoalStatus.Status.SUCCESS
            ):
                logger.error("Failed to set goal")
    else:
        logger.error("Failed to move spine")

def get_arm_state(mmk2, arm):
    robot_state = mmk2.get_robot_state()
    if robot_state is None:
        logger.error("Failed to get robot state")
        return
    left_joint_name2idx = {f"left_arm_joint_{i}": idx for i in range(1, 7) for idx, name in
                           enumerate(robot_state.joint_state.name) if name == f"left_arm_joint{i}"}
    left_joint_pos = np.array([robot_state.joint_state.position[idx] for _, idx in left_joint_name2idx.items()])

    right_joint_name2idx = {f"right_arm_joint_{i}": idx for i in range(1, 7) for idx, name in
                            enumerate(robot_state.joint_state.name) if name == f"right_arm_joint{i}"}
    right_joint_pos = np.array([robot_state.joint_state.position[idx] for _, idx in right_joint_name2idx.items()])

    if arm == "left":
        return robot_state.robot_pose.robot_pose["left_arm"], left_joint_pos
    elif arm == "right":
        return robot_state.robot_pose.robot_pose["right_arm"], right_joint_pos