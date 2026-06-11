"""좌표 변환 수치 검증 — 헤드셋 없이 frames.py 정확성 확인."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from groot_teleop.common import frames


def test_headset_to_world_axes():
    """WebXR 축 → z-up world 축 매핑이 PicoStreamer 상수와 일치."""
    # WebXR +Y(위) → world +Z(위)
    T = np.eye(4); T[:3, 3] = [0, 1, 0]
    out = frames.headset_to_world(T)
    np.testing.assert_allclose(out[:3, 3], [0, 0, 1], atol=1e-9)

    # WebXR -Z(정면) → world +X(정면)
    T = np.eye(4); T[:3, 3] = [0, 0, -1]
    out = frames.headset_to_world(T)
    np.testing.assert_allclose(out[:3, 3], [1, 0, 0], atol=1e-9)

    # WebXR +X(오른쪽) → world -Y(오른쪽)
    T = np.eye(4); T[:3, 3] = [1, 0, 0]
    out = frames.headset_to_world(T)
    np.testing.assert_allclose(out[:3, 3], [0, -1, 0], atol=1e-9)


def test_headset_to_world_preserves_se3():
    rng = np.random.default_rng(0)
    for _ in range(20):
        T = np.eye(4)
        T[:3, :3] = R.random(random_state=rng.integers(1 << 30)).as_matrix()
        T[:3, 3] = rng.normal(size=3)
        out = frames.headset_to_world(T)
        assert frames._is_valid_se3(out), "변환 결과가 SE(3) 가 아님"


def test_wrist_relative_zero_when_coincident():
    """손목 == 머리 pose 이면 상대 위치 0 (회전 identity 머리)."""
    head = np.eye(4); head[:3, 3] = [0, 1.5, 0]
    wrist = head.copy()
    out = frames.wrist_relative_to_head(wrist, head)
    np.testing.assert_allclose(out[:3, 3], [0, 0, 0], atol=1e-9)


def test_wrist_relative_translation_only():
    """머리 yaw=0 일 때 손목이 머리보다 정면(-Z) 30cm 앞 → world +X 0.3."""
    head = np.eye(4); head[:3, 3] = [0, 1.5, 0]
    wrist = np.eye(4); wrist[:3, 3] = [0, 1.5, -0.3]   # 정면 30cm
    out = frames.wrist_relative_to_head(wrist, head)
    np.testing.assert_allclose(out[:3, 3], [0.3, 0, 0], atol=1e-9)


def test_wrist_relative_yaw_compensation():
    """사용자가 머리를 yaw 90° 돌리고 같은 '정면' 으로 손을 뻗으면,
    yaw 보상 후 손목 target 은 머리 방향과 무관하게 정면(+X)이어야 한다."""
    yaw = np.pi / 2
    Rz = R.from_euler("z", yaw).as_matrix()  # world 기준 머리 yaw
    # 머리: world 에서 yaw 90° 회전. WebXR frame 으로 역변환해 입력 구성.
    head_w = np.eye(4); head_w[:3, :3] = Rz; head_w[:3, 3] = [0, 0, 1.5]  # z-up world
    # 머리가 바라보는 정면으로 0.3m 앞 손목 (world).
    forward_w = Rz @ np.array([0.3, 0, 0])
    wrist_w = np.eye(4); wrist_w[:3, :3] = Rz; wrist_w[:3, 3] = head_w[:3, 3] + forward_w

    # world(z-up) → WebXR(y-up) 역변환해 입력으로 사용.
    Rinv = frames.R_HEADSET_TO_WORLD.T
    def to_xr(Tw):
        Tx = np.eye(4)
        Tx[:3, :3] = Rinv @ Tw[:3, :3] @ frames.R_HEADSET_TO_WORLD
        Tx[:3, 3] = Rinv @ Tw[:3, 3]
        return Tx

    out = frames.wrist_relative_to_head(to_xr(wrist_w), to_xr(head_w))
    # yaw 보상 후 '정면' 은 항상 world +X.
    np.testing.assert_allclose(out[:3, 3], [0.3, 0, 0], atol=1e-6)


def test_relative_motion_identity_origin():
    """origin == robot_origin == identity 면 target == 현재 손목."""
    T_now = np.eye(4); T_now[:3, 3] = [0.1, 0.2, 0.3]
    out = frames.relative_motion(T_now, np.eye(4), np.eye(4))
    np.testing.assert_allclose(out, T_now, atol=1e-12)


def test_relative_motion_adds_delta_to_robot_origin():
    T_origin = np.eye(4); T_origin[:3, 3] = [1, 0, 0]
    T_now = np.eye(4); T_now[:3, 3] = [1.05, 0, 0]      # +5cm
    T_robot = np.eye(4); T_robot[:3, 3] = [0.5, 0.5, 0.5]
    out = frames.relative_motion(T_now, T_origin, T_robot)
    np.testing.assert_allclose(out[:3, 3], [0.55, 0.5, 0.5], atol=1e-12)


def test_pose_from_xyz_quat_zero_quat_identity():
    T = frames.pose_from_xyz_quat([1, 2, 3], [0, 0, 0, 0])
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(T[:3, 3], [1, 2, 3], atol=1e-12)
