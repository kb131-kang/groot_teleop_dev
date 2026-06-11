"""GalaxyXRStreamer 단위테스트 — SyntheticPoseSource 주입, 헤드셋 불필요."""

import numpy as np
import pytest

from groot_teleop.input.base_streamer import StreamerOutput
from groot_teleop.input.galaxy_xr_streamer import GalaxyXRStreamer, PINCH_CLOSE_M
from groot_teleop.input.synthetic_streamer import SyntheticPoseSource


@pytest.fixture
def streamer():
    src = SyntheticPoseSource(use_hand_tracking=True)
    return GalaxyXRStreamer(pose_source=src)


def test_output_contract(streamer):
    """PicoStreamer 와 동일한 StreamerOutput 키/형상."""
    out = streamer.get()
    assert isinstance(out, StreamerOutput)
    assert out.source == "galaxy_xr"

    # ik_data
    for k in ("left_wrist", "right_wrist", "left_fingers", "right_fingers"):
        assert k in out.ik_data
    assert out.ik_data["left_wrist"].shape == (4, 4)
    assert out.ik_data["right_wrist"].shape == (4, 4)
    assert out.ik_data["left_fingers"]["position"].shape == (25, 4, 4)

    # control_data
    assert "base_height_command" in out.control_data
    assert len(out.control_data["navigate_cmd"]) == 3
    assert "toggle_policy_action" in out.control_data

    # teleop / data_collection
    assert "toggle_activation" in out.teleop_data
    assert "toggle_data_collection" in out.data_collection_data
    assert "toggle_data_abort" in out.data_collection_data


def test_wrist_is_valid_se3(streamer):
    from groot_teleop.common import frames

    out = streamer.get()
    assert frames._is_valid_se3(out.ik_data["left_wrist"])
    assert frames._is_valid_se3(out.ik_data["right_wrist"])


def test_base_height_default(streamer):
    out = streamer.get()
    assert out.control_data["base_height_command"] == pytest.approx(0.74)


def test_pinch_drives_gripper():
    """pinch 거리 < threshold → index tip(인덱스 9) close 마킹."""
    src = SyntheticPoseSource(use_hand_tracking=True)
    streamer = GalaxyXRStreamer(pose_source=src)

    # pinch 닫힘(거리 0) 강제.
    src.set_time(0.0)
    # SyntheticPoseSource._pinch_value(0) = 0.04*(1+sin(0)) = 0.04 → 열림
    out_open = streamer.get()
    assert out_open.ik_data["left_fingers"]["position"][9, 0, 3] == 0.0

    # pinch 값이 PINCH_CLOSE_M 미만이 되는 시점 찾기 (sin 이 음수 구간).
    # t = 3*0.75 = 2.25s → sin(2π*2.25/3)=sin(1.5π)=-1 → 0.04*(1-1)=0 → 닫힘
    src.set_time(2.25)
    out_close = streamer.get()
    assert out_close.ik_data["left_fingers"]["position"][9, 0, 3] == 1.0
    # thumb tip 은 항상 open.
    assert out_close.ik_data["left_fingers"]["position"][4, 0, 3] == 1.0


def test_synthetic_determinism():
    """같은 t → 같은 출력 (재현성)."""
    src = SyntheticPoseSource()
    src.set_time(1.234)
    a = np.array(src.right_arm_pose)
    src.set_time(9.9)
    src.set_time(1.234)
    b = np.array(src.right_arm_pose)
    np.testing.assert_allclose(a, b)


def test_motion_changes_over_time(streamer):
    """시간이 흐르면 손목 target 이 실제로 움직인다 (정적 아님)."""
    streamer.source.set_time(0.0)
    p0 = streamer.get().ik_data["right_wrist"][:3, 3].copy()
    streamer.source.set_time(1.0)
    p1 = streamer.get().ik_data["right_wrist"][:3, 3].copy()
    assert np.linalg.norm(p1 - p0) > 1e-3


def test_all_zero_pose_sanitized():
    """초기 all-zero pose frame 에서도 crash 없이 identity 손목 반환."""

    class ZeroSource:
        use_hand_tracking = True
        head_pose = np.zeros((4, 4))
        left_arm_pose = np.zeros((4, 4))
        right_arm_pose = np.zeros((4, 4))
        left_hand_pinchValue = 1.0
        right_hand_pinchValue = 1.0

    streamer = GalaxyXRStreamer(pose_source=ZeroSource())
    out = streamer.get()
    np.testing.assert_allclose(out.ik_data["left_wrist"], np.eye(4), atol=1e-9)
