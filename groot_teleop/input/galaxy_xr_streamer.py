"""GalaxyXRStreamer — Galaxy XR / Quest 3 → GR00T StreamerOutput.

GR00T-WholeBodyControl 의 ``BaseStreamer`` 를 구현해, PICO 전용 ``PicoStreamer``
를 **무변경 drop-in** 으로 대체한다. 입력원은 teleop_dev 의 ``BridgePoseStore``
(WebXR ws bridge) 이며, 테스트 시에는 ``SyntheticPoseSource`` 를 주입한다.

GR00T 통합 방법 (TeleopStreamer 측 1줄 추가):
    elif body_control_device == "galaxy_xr":
        from groot_teleop.input.galaxy_xr_streamer import GalaxyXRStreamer
        self.body_streamer = GalaxyXRStreamer()
        self.body_streamer.start_streaming()

출력 계약 (PicoStreamer 와 동일):
    ik_data      : left_wrist(4×4), right_wrist(4×4), left/right_fingers.position
    control_data : base_height_command, navigate_cmd[vx,vy,wz], toggle_policy_action
    teleop_data  : toggle_activation
    data_collection_data : toggle_data_collection, toggle_data_abort

손목 pose 는 ``frames.wrist_relative_to_head`` 로 headset-relative + yaw 보상
(PicoStreamer._process_xr_pose 와 수치 동일). 손가락/그리퍼는 WebXR pinch 값
으로 구동 (PICO 의 trigger/grip 대체).
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from groot_teleop.common import frames
from groot_teleop.input.base_streamer import BaseStreamer, StreamerOutput

# 그리퍼: pinch 거리(m) < CLOSE → 닫힘. BridgePoseStore PINCH_THRESHOLD(0.01)
# 보다 크게 잡아 controller 없이도 안정적 토글.
PINCH_CLOSE_M = 0.03
BASE_HEIGHT_DEFAULT = 0.74  # G1 서있는 높이 (PicoStreamer 와 동일)
BASE_HEIGHT_MIN = 0.2
BASE_HEIGHT_MAX = 0.74


class GalaxyXRStreamer(BaseStreamer):
    def __init__(
        self,
        pose_source: Optional[Any] = None,
        use_hand_tracking: bool = True,
        bridge_port: Optional[int] = None,
    ):
        """
        Parameters
        ----------
        pose_source : BridgePoseStore 호환 객체 또는 None
            None 이면 BridgePoseStore 를 생성(실제 헤드셋 ws). 테스트 시
            SyntheticPoseSource 주입.
        use_hand_tracking : bool
            hand-tracking(pinch) 모드. False 면 controller 모드(미구현 — 향후).
        bridge_port : int | None
            BridgePoseStore ws port (None=config/8013).
        """
        self.use_hand_tracking = use_hand_tracking
        self._owns_source = pose_source is None
        if pose_source is None:
            from groot_teleop.input.bridge import BridgePoseStore

            pose_source = BridgePoseStore(
                use_hand_tracking=use_hand_tracking, port=bridge_port
            )
        self.source = pose_source
        self.reset_status()

    # ── BaseStreamer 인터페이스 ────────────────────────────────────────
    def reset_status(self):
        self.current_base_height = BASE_HEIGHT_DEFAULT
        self._last_left_pinch_closed = False
        self._last_right_pinch_closed = False

    def start_streaming(self):
        # BridgePoseStore 는 생성 시 ws server 가 이미 떠 있음 — noop.
        pass

    def stop_streaming(self):
        if self._owns_source and hasattr(self.source, "close"):
            self.source.close()

    def get(self) -> StreamerOutput:
        head_T_xr = np.asarray(self.source.head_pose, dtype=np.float64)
        left_T_xr = np.asarray(self.source.left_arm_pose, dtype=np.float64)
        right_T_xr = np.asarray(self.source.right_arm_pose, dtype=np.float64)

        # 아직 pose 가 안 들어온 (all-zero) frame 방어 — identity 로 대체.
        head_T_xr = _sanitize(head_T_xr)
        left_T_xr = _sanitize(left_T_xr)
        right_T_xr = _sanitize(right_T_xr)

        left_wrist = frames.wrist_relative_to_head(left_T_xr, head_T_xr)
        right_wrist = frames.wrist_relative_to_head(right_T_xr, head_T_xr)

        left_fingers = self._finger_data("left")
        right_fingers = self._finger_data("right")

        return StreamerOutput(
            ik_data={
                "left_wrist": left_wrist,
                "right_wrist": right_wrist,
                "left_fingers": {"position": left_fingers},
                "right_fingers": {"position": right_fingers},
            },
            control_data={
                "base_height_command": self.current_base_height,
                # 손-추적 모드에는 joystick 이동 입력이 없음 → 정지. 보행 명령은
                # Phase 3 에서 별도 입력(thumbstick/Vive 발목)으로 결선 예정.
                "navigate_cmd": [0.0, 0.0, 0.0],
                "toggle_policy_action": False,
            },
            teleop_data={
                "toggle_activation": False,
            },
            data_collection_data={
                "toggle_data_collection": False,
                "toggle_data_abort": False,
            },
            source="galaxy_xr",
        )

    # ── 손가락(그리퍼) ─────────────────────────────────────────────────
    def _finger_data(self, hand: str) -> np.ndarray:
        """PicoStreamer 와 동일한 25×4×4 fingertip 텐서.

        index 4 = thumb tip(항상 open), index 9 = index tip. pinch 닫힘이면
        index tip 을 close 로 마킹 → 다운스트림 gripper IK 가 닫음.
        """
        fingertips = np.zeros([25, 4, 4])
        fingertips[4, 0, 3] = 1.0  # thumb open
        pinch_v = getattr(self.source, f"{hand}_hand_pinchValue", 1.0)
        if float(pinch_v) < PINCH_CLOSE_M:
            fingertips[9, 0, 3] = 1.0  # index tip close → grip
        return fingertips

    def __del__(self):
        pass


def _sanitize(T: np.ndarray) -> np.ndarray:
    """all-zero / 비정상 4×4 를 identity 로 대체 (초기 frame 방어)."""
    if T.shape != (4, 4) or np.allclose(T, 0.0):
        return np.eye(4)
    # 회전부가 0 이면 identity 회전 강제.
    if np.allclose(T[:3, :3], 0.0):
        T = T.copy()
        T[:3, :3] = np.eye(3)
        T[3, :] = [0, 0, 0, 1]
    return T


__all__ = ["GalaxyXRStreamer", "PINCH_CLOSE_M"]
