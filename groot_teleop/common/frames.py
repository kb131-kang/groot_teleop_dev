"""좌표계 변환 — WebXR pose → GR00T z-up, headset-relative 손목 frame.

배경
----
GR00T ``PicoStreamer._process_xr_pose`` 는 PICO 의 (xyz, quat) 입력을 받아
  1. y-up(WebXR/headset) → z-up(robot world) 회전,
  2. headset 기준 상대 위치(delta),
  3. headset yaw 보상(사용자가 어느 방향을 보든 정면 기준 정렬),
순으로 손목 4×4 SE(3) 를 만든다.

teleop_dev 의 ``BridgePoseStore`` 는 같은 정보를 (xyz,quat) 가 아니라 **4×4 행렬**
로 제공한다 (WebXR local-floor reference space). 본 모듈은 PicoStreamer 와
**수치적으로 동일한 결과**를 내도록 그 로직을 4×4 입력용으로 이식하고, 순수
함수로 분리해 헤드셋 없이 단위테스트 가능하게 한다.

좌표계 규약
----------
WebXR (local-floor):  +X 오른쪽, +Y 위, -Z 정면 (사용자가 보는 방향)
GR00T world (z-up):   +X 정면, +Y 왼쪽, +Z 위

R_HEADSET_TO_WORLD 는 PicoStreamer 의 상수와 동일하다.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

# PicoStreamer 와 동일한 상수 (y-up headset → z-up world).
R_HEADSET_TO_WORLD = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float64,
)


def _is_valid_se3(T: np.ndarray) -> bool:
    """4×4 가 유효한 SE(3) 인지 (마지막 행 [0,0,0,1], R 직교) 대략 확인."""
    if T.shape != (4, 4):
        return False
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=1e-6):
        return False
    Rm = T[:3, :3]
    return bool(np.allclose(Rm @ Rm.T, np.eye(3), atol=1e-3))


def headset_to_world(T_xr: np.ndarray) -> np.ndarray:
    """WebXR(y-up) 4×4 pose → z-up world 4×4 pose.

    위치:  p' = R @ p
    회전:  Rr' = R @ Rr @ Rᵀ   (basis change)
    """
    T_xr = np.asarray(T_xr, dtype=np.float64)
    out = np.eye(4)
    out[:3, :3] = R_HEADSET_TO_WORLD @ T_xr[:3, :3] @ R_HEADSET_TO_WORLD.T
    out[:3, 3] = R_HEADSET_TO_WORLD @ T_xr[:3, 3]
    return out


def yaw_of(T: np.ndarray) -> float:
    """z-up frame 의 yaw(Z축 회전, rad). euler 'xyz' 의 마지막 성분."""
    return float(R.from_matrix(np.asarray(T)[:3, :3]).as_euler("xyz")[2])


def wrist_relative_to_head(
    T_wrist_xr: np.ndarray,
    T_head_xr: np.ndarray,
) -> np.ndarray:
    """손목 WebXR pose → headset 기준 yaw-보상된 z-up 손목 4×4.

    PicoStreamer._process_xr_pose 와 동일한 절차:
      1. 손목/머리 모두 z-up 으로 변환.
      2. 위치 delta = 손목 - 머리 (z-up).
      3. 머리 yaw 의 역회전을 delta 와 손목 회전 양쪽에 적용.

    Returns
    -------
    (4,4) SE(3) — 정면(머리 yaw=0) 기준으로 정렬된 손목 target pose.
    """
    T_wrist_w = headset_to_world(T_wrist_xr)
    T_head_w = headset_to_world(T_head_xr)

    delta = T_wrist_w[:3, 3] - T_head_w[:3, 3]

    head_yaw = yaw_of(T_head_w)
    inv_yaw = R.from_euler("z", -head_yaw).as_matrix()

    out = np.eye(4)
    out[:3, :3] = inv_yaw @ T_wrist_w[:3, :3]
    out[:3, 3] = inv_yaw @ delta
    return out


def pose_from_xyz_quat(xyz, quat_xyzw) -> np.ndarray:
    """(xyz, quat[x,y,z,w]) → 4×4. quat 이 all-zero 면 identity 로 대체.

    PicoStreamer 가 PICO 입력에서 쓰던 방어 로직과 동일 — BridgePoseStore 의
    4×4 가 아직 안 들어온 (all-zero) 경우에도 호출측 코드가 안전하게 동작.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    quat = np.asarray(quat_xyzw, dtype=np.float64)
    if np.allclose(quat, 0):
        quat = np.array([0.0, 0.0, 0.0, 1.0])
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat).as_matrix()
    T[:3, 3] = xyz
    return T


def relative_motion(
    T_now: np.ndarray,
    T_origin: np.ndarray,
    T_robot_origin: np.ndarray,
) -> np.ndarray:
    """relative-motion 매핑 — recalibrate 시점(origin) 대비 손목 변위를
    로봇 origin TCP 에 더해 target TCP 를 만든다.

    teleop_dev/xr_sender (XRRelativeFrameAligner) 의 핵심 모델:
        T_target = T_robot_origin · (T_origin⁻¹ · T_now)

    사용자가 'r'(recalibrate) 를 누른 순간의 손목 pose(T_origin)과 로봇 TCP
    (T_robot_origin)을 캡처하고, 이후 손목의 상대 변위만 로봇에 전달한다.
    이로써 사용자가 손을 편한 위치에 두고 시작할 수 있다(absolute 매핑의 점프 방지).
    """
    rel = np.linalg.inv(np.asarray(T_origin)) @ np.asarray(T_now)
    return np.asarray(T_robot_origin) @ rel


__all__ = [
    "R_HEADSET_TO_WORLD",
    "headset_to_world",
    "yaw_of",
    "wrist_relative_to_head",
    "pose_from_xyz_quat",
    "relative_motion",
    "_is_valid_se3",
]
