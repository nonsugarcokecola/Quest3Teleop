import logging
import time

import numpy as np
from mmk2_types.types import MMK2Components
from mmk2_types.grpc_msgs import (
    JointState,
    ForwardPositionParams,
    TrajectoryParams,
    MoveServoParams,
    GoalStatus,
    BaseControlParams,
    BuildMapParams,
    Pose,
    Twist3D,
    BaseChargeStationParams,
    ArrayStamped,
    Position,
    Orientation
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def control_arm_poses(mmk2, stop_arm_pose, stop_quat,arm_name):
    if arm_name == "l":
        trgt_pos_base_link = {
            MMK2Components.LEFT_ARM: Pose(
                position=Position(x=stop_arm_pose[0], y=stop_arm_pose[1],
                                  z=stop_arm_pose[2]),
                orientation=Orientation(x=0.500, y=0.497, z=-0.506, w=0.496),
            ),
        }
    elif arm_name == "r":
        trgt_pos_base_link = {
            MMK2Components.RIGHT_ARM: Pose(
                position=Position(x=stop_arm_pose[0], y=stop_arm_pose[1],
                                  z=stop_arm_pose[2]),
                orientation=Orientation(x=-0.500, y=0.497, z=0.506, w=0.496),
            ),
        }
    if (
        mmk2.set_goal(trgt_pos_base_link, TrajectoryParams()).value
        != GoalStatus.Status.SUCCESS
    ):
        logger.error("Failed to set poses")

def control_traj_servo_separate(mmk2, trgt_joint_action):
    action_ref = trgt_joint_action.copy()
    if (
        mmk2.set_goal(
            {MMK2Components.SPINE: action_ref.pop(MMK2Components.SPINE)},
            TrajectoryParams(),
        ).value
        == GoalStatus.Status.SUCCESS
    ):
        if (
            mmk2.set_goal(action_ref, ForwardPositionParams()).value
            != GoalStatus.Status.SUCCESS
        ):
            logger.error("Failed to set goal")
    else:
        logger.error("Failed to move spine")


def control_gripper_servo_separate(mmk2, trgt_joint_action):
    action_ref = trgt_joint_action.copy()
    if (
        mmk2.set_goal(action_ref, ForwardPositionParams()).value
        != GoalStatus.Status.SUCCESS
    ):
        logger.info("Success to set the gripper goal")
    else:
        logger.error("Failed to set gripper")

def control_spine_servo_separate(mmk2, trgt_joint_action):
    action_ref = trgt_joint_action.copy()
    if (
        mmk2.set_goal(
            {MMK2Components.SPINE: action_ref.pop(MMK2Components.SPINE)},
            ForwardPositionParams(),
        ).value
        == GoalStatus.Status.SUCCESS
    ):
        logger.info("Successfully moved spine")
    else:
        logger.error("Failed to move spine")

def control_arm_servo_separate(mmk2, trgt_joint_action):
    action_ref = trgt_joint_action.copy()
    if (
        mmk2.set_goal(
            action_ref,
            ForwardPositionParams(),
        ).value
        == GoalStatus.Status.SUCCESS
    ):
        logger.info("Successfully moved the arm")
    else:
        logger.error("Failed to move arm")

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


# def control_left_gripper_servo_separate(mmk2, trgt_joint_action):
#     action_ref = trgt_joint_action.copy()
#     if (
#         mmk2.set_goal(
#             {MMK2Components.LEFT_ARM_EEF: action_ref.pop(MMK2Components.LEFT_ARM_EEF)},
#             ForwardPositionParams(),
#         ).value
#         == GoalStatus.Status.SUCCESS
#     ):
#         logger.info("Successfully left gripper")
#     else:
#         logger.error("Failed to left gripper")
#
# def control_right_gripper_servo_separate(mmk2, trgt_joint_action):
#     action_ref = trgt_joint_action.copy()
#     if (
#         mmk2.set_goal(
#             {MMK2Components.RIGHT_ARM_EEF: action_ref.pop(MMK2Components.RIGHT_ARM_EEF)},
#             ForwardPositionParams(),
#         ).value
#         == GoalStatus.Status.SUCCESS
#     ):
#         logger.info("Successfully right gripper")
#     else:
#         logger.error("Failed to right gripper")
