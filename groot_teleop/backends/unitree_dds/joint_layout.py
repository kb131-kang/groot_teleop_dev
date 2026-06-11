"""Unitree G1 29-DoF 관절 레이아웃 — DDS lowcmd 인덱스 규약.

unitree_sim_isaaclab ``action_provider_dds.py`` 및 실제 G1 ``rt/lowcmd``
(unitree_hg.LowCmd_) 의 motor_cmd 순서와 1:1 일치한다.

lowcmd motor_cmd[] 인덱스 (29 DoF):
    0–5    left leg  : hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
    6–11   right leg : (동일 순서)
    12–14  waist     : yaw, roll, pitch
    15–21  left arm  : shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
                       wrist_roll, wrist_pitch, wrist_yaw
    22–28  right arm : (동일 순서)

teleop(상체) 는 15–28 의 14개 팔 관절만 구동하고, 0–14(다리+허리)는
하체 정책(SONIC) 또는 hold(기본 자세 유지)에 맡긴다.
"""

from __future__ import annotations

NUM_JOINTS = 29

LEFT_LEG = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
]
RIGHT_LEG = [
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
LEFT_ARM = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
RIGHT_ARM = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# lowcmd 순서대로 전체 29개.
JOINT_NAMES = LEFT_LEG + RIGHT_LEG + WAIST + LEFT_ARM + RIGHT_ARM
assert len(JOINT_NAMES) == NUM_JOINTS, len(JOINT_NAMES)

NAME_TO_INDEX = {n: i for i, n in enumerate(JOINT_NAMES)}

LEFT_ARM_INDICES = [NAME_TO_INDEX[n] for n in LEFT_ARM]    # [15..21]
RIGHT_ARM_INDICES = [NAME_TO_INDEX[n] for n in RIGHT_ARM]  # [22..28]
ARM_INDICES = LEFT_ARM_INDICES + RIGHT_ARM_INDICES         # [15..28]
LOWER_BODY_INDICES = [NAME_TO_INDEX[n] for n in (LEFT_LEG + RIGHT_LEG + WAIST)]  # [0..14]

# EE(IK target) frame — g1_29dof_with_hand.urdf 의 palm link.
LEFT_EE_FRAME = "left_hand_palm_link"
RIGHT_EE_FRAME = "right_hand_palm_link"

# 기본 PD 게인 (팔, 위치 제어). 실 로봇/sim 튜닝값은 USER_TEST 단계에서 조정.
DEFAULT_ARM_KP = 60.0
DEFAULT_ARM_KD = 1.5

__all__ = [
    "NUM_JOINTS", "JOINT_NAMES", "NAME_TO_INDEX",
    "LEFT_ARM", "RIGHT_ARM", "LEFT_ARM_INDICES", "RIGHT_ARM_INDICES",
    "ARM_INDICES", "LOWER_BODY_INDICES",
    "LEFT_EE_FRAME", "RIGHT_EE_FRAME",
    "DEFAULT_ARM_KP", "DEFAULT_ARM_KD",
]
