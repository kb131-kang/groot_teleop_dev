"""G1DDSBridge 단위테스트 — DDS/sim/헤드셋 무관 (RecordingPublisher).

전체 경로 B 파이프라인을 헤드셋 없이 검증:
    SyntheticPoseSource → GalaxyXRStreamer → G1DDSBridge → RecordingPublisher
"""

import numpy as np
import pytest

from groot_teleop.backends.unitree_dds import joint_layout as L
from groot_teleop.backends.unitree_dds.dds_bridge import (
    RecordingPublisher,
    IKTargetMapper,
)

pytest.importorskip("pinocchio")
from groot_teleop.backends.unitree_dds.g1_arm_ik import default_urdf_path  # noqa: E402

if not default_urdf_path().exists():
    pytest.skip("G1 URDF 없음", allow_module_level=True)

from groot_teleop.backends.unitree_dds.dds_bridge import G1DDSBridge  # noqa: E402
from groot_teleop.backends.unitree_dds.g1_arm_ik import IKConfig  # noqa: E402
from groot_teleop.input.galaxy_xr_streamer import GalaxyXRStreamer  # noqa: E402
from groot_teleop.input.synthetic_streamer import SyntheticPoseSource  # noqa: E402


# ── joint_layout 순수 검증 (URDF 무관이지만 같이 둠) ────────────────────
def test_joint_layout_indices():
    assert L.NUM_JOINTS == 29
    assert len(L.JOINT_NAMES) == 29
    assert L.LEFT_ARM_INDICES == list(range(15, 22))
    assert L.RIGHT_ARM_INDICES == list(range(22, 29))
    assert L.ARM_INDICES == list(range(15, 29))
    assert L.LOWER_BODY_INDICES == list(range(0, 15))
    # 중복/누락 없음.
    assert sorted(L.ARM_INDICES + L.LOWER_BODY_INDICES) == list(range(29))


@pytest.fixture
def bridge():
    pub = RecordingPublisher()
    # reachable 보장 위해 mapper origin 을 팔 작업공간 중심 근처로.
    mapper = IKTargetMapper(scale=0.5)
    br = G1DDSBridge(
        publisher=pub,
        mapper=mapper,
        ik_config=IKConfig(max_iters=100, eps=1e-3, n_restarts=2),
    )
    return br, pub


def test_publishes_29dof(bridge):
    br, pub = bridge
    streamer = GalaxyXRStreamer(pose_source=SyntheticPoseSource())
    out = streamer.get()
    info = br.step(out)
    assert pub.last is not None
    assert pub.last["positions"].shape == (29,)
    assert pub.last["kp"].shape == (29,)


def test_lower_body_held_arms_driven(bridge):
    br, pub = bridge
    src = SyntheticPoseSource()
    streamer = GalaxyXRStreamer(pose_source=src)
    src.set_time(1.0)
    br.step(streamer.get())
    pos = pub.last["positions"]
    # 하체+허리(0–14)는 hold(0) 유지.
    np.testing.assert_allclose(pos[L.LOWER_BODY_INDICES], 0.0, atol=1e-12)
    # 팔 게인만 활성.
    assert np.all(pub.last["kp"][L.ARM_INDICES] > 0)
    np.testing.assert_allclose(pub.last["kp"][L.LOWER_BODY_INDICES], 0.0)


def test_arms_track_motion(bridge):
    """손목 target 이 움직이면 팔 관절 명령도 변한다 (정적 아님)."""
    br, pub = bridge
    src = SyntheticPoseSource()
    streamer = GalaxyXRStreamer(pose_source=src)
    src.set_time(0.0)
    br.step(streamer.get())
    q0 = pub.last["positions"][L.RIGHT_ARM_INDICES].copy()
    src.set_time(1.5)
    br.step(streamer.get())
    q1 = pub.last["positions"][L.RIGHT_ARM_INDICES].copy()
    assert np.linalg.norm(q1 - q0) > 1e-3


def test_warm_start_converges_on_reachable_targets():
    """reachable target(팔 FK 로 생성)을 따라가는 연속 궤적에서 IK 가 수렴.

    identity mapper(target = wrist_T 직접)로 두고, 양 팔의 도달 가능한 palm
    pose 를 작은 궤적으로 생성해 입력. warm-start 가 동작하면 거의 모든
    프레임이 수렴해야 한다. (synthetic 손 workspace ↔ 로봇 팔 reachable set
    정합은 calibration=USER_TEST 사안이므로, 여기선 reachability 를 보장.)
    """
    from types import SimpleNamespace
    from groot_teleop.backends.unitree_dds.g1_arm_ik import G1ArmIK

    pub = RecordingPublisher()
    br = G1DDSBridge(
        publisher=pub,
        mapper=IKTargetMapper(
            left_origin=np.zeros(3), right_origin=np.zeros(3), scale=1.0
        ),
        ik_config=IKConfig(max_iters=100, eps=1e-3, n_restarts=2),
    )
    lik, rik = G1ArmIK("left"), G1ArmIK("right")
    ql0, qr0 = np.zeros(7), np.zeros(7)

    n_conv = 0
    N = 20
    for k in range(N):
        # 중립 근처에서 부드럽게 변하는 reachable 자세 → FK → target palm pose.
        ql = ql0 + 0.2 * np.sin(0.1 * k + np.arange(7))
        qr = qr0 + 0.2 * np.sin(0.1 * k + np.arange(7) + 1.0)
        left_T = lik.fk(lik.clamp_to_limits(ql)).homogeneous
        right_T = rik.fk(rik.clamp_to_limits(qr)).homogeneous
        out = SimpleNamespace(ik_data={"left_wrist": left_T, "right_wrist": right_T})
        info = br.step(out)
        n_conv += int(info["ik_converged"][0] and info["ik_converged"][1])

    assert n_conv >= N - 2, f"reachable 궤적 수렴 {n_conv}/{N}"


# ── 개선 패스: 안전 clamp / relative-motion / hold ──────────────────────
def test_workspace_clamp_bounds_target():
    """envelope 밖 target 위치가 G1 한계로 clamp 된다."""
    from groot_teleop.backends.unitree_dds.dds_bridge import g1_workspace_limits

    mapper = IKTargetMapper(
        left_origin=np.zeros(3), right_origin=np.zeros(3), scale=1.0
    )
    far = np.eye(4)
    far[:3, 3] = [10.0, 10.0, 10.0]   # 한참 밖
    T = mapper.map(far, "left")
    lim = g1_workspace_limits()
    assert T[0, 3] <= lim.x_max + 1e-9
    assert T[1, 3] <= lim.y_max + 1e-9
    assert T[2, 3] <= lim.z_max + 1e-9


def test_relative_mode_origin_invariant():
    """relative 모드에서 출력은 mapper origin 에 불변 (recalibrate 가 상쇄)."""
    from types import SimpleNamespace

    def run_with_origin(origin_val):
        pub = RecordingPublisher()
        br = G1DDSBridge(
            publisher=pub,
            mapper=IKTargetMapper(
                left_origin=np.full(3, origin_val),
                right_origin=np.full(3, origin_val),
                scale=1.0,
                limits=None,
            ),
            ik_config=IKConfig(max_iters=80, eps=1e-3, n_restarts=1),
            relative=True,
        )
        # 동일 로봇 상태 가정: robot_origin 은 robot state(여기선 q=0)에서 옴 —
        # mapper origin 과 무관. (실사용은 로봇 피드백 q.)
        br._q_left = np.zeros(br.left_ik.nq)
        br._q_right = np.zeros(br.right_ik.nq)
        w0 = np.eye(4); w0[:3, 3] = [0.0, 0.0, 0.0]
        out0 = SimpleNamespace(ik_data={"left_wrist": w0, "right_wrist": w0})
        br.recalibrate(out0)     # origin 캡처 (robot_origin = FK(0), 동일)
        w1 = np.eye(4); w1[:3, 3] = [0.05, 0.0, 0.0]
        out1 = SimpleNamespace(ik_data={"left_wrist": w1, "right_wrist": w1})
        br.step(out1)
        return pub.last["positions"][L.LEFT_ARM_INDICES].copy()

    a = run_with_origin(0.0)
    b = run_with_origin(0.3)   # 다른 mapper origin
    np.testing.assert_allclose(a, b, atol=1e-3)


def test_hold_keeps_last_arm():
    from types import SimpleNamespace

    pub = RecordingPublisher()
    br = G1DDSBridge(
        publisher=pub,
        mapper=IKTargetMapper(scale=0.3),
        ik_config=IKConfig(max_iters=50, eps=1e-2, n_restarts=1),
    )
    src = SyntheticPoseSource(); src.set_time(0.5)
    streamer = GalaxyXRStreamer(pose_source=src)
    br.step(streamer.get())
    last_arm = pub.last["positions"][L.ARM_INDICES].copy()
    info = br.hold()
    assert info["held"] is True
    np.testing.assert_allclose(pub.last["positions"][L.ARM_INDICES], last_arm)
