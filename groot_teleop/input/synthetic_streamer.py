"""SyntheticPoseSource — 헤드셋 없이 XR pose 를 합성해 파이프라인을 구동.

용도
----
실제 Galaxy XR / Quest 3 없이도 ``GalaxyXRStreamer`` → IK → sim 백엔드 전체를
돌려보기 위한 입력원. ``BridgePoseStore`` 와 **동일한 read-only property
인터페이스**(head_pose / left_arm_pose / right_arm_pose / *_hand_pinchValue 등)
를 제공하므로 GalaxyXRStreamer 에 그대로 주입(inject)할 수 있다.

움직임 모델
----------
- head: local-floor 원점에 고정(약간의 yaw 흔들림 옵션).
- 양 손목: 가슴 앞쪽에서 원/사인 궤적으로 부드럽게 이동 (WebXR y-up frame).
- pinch: 주기적으로 열고/닫음 → 그리퍼 토글 테스트.

시간은 외부에서 ``t`` 를 주거나(deterministic, 테스트용), 미지정 시 내부
단조 카운터를 사용(실시간 흉내). ``Math.random``/wall-clock 없이도 재현 가능.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


def _se3(xyz, rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return T


class SyntheticPoseSource:
    """BridgePoseStore 호환 합성 pose 제공자 (하드웨어 무관 테스트용)."""

    def __init__(
        self,
        use_hand_tracking: bool = True,
        hand_radius_m: float = 0.15,
        hand_period_s: float = 4.0,
        pinch_period_s: float = 3.0,
        head_yaw_amp_rad: float = 0.0,
    ):
        self.use_hand_tracking = use_hand_tracking
        self.hand_radius_m = hand_radius_m
        self.hand_period_s = hand_period_s
        self.pinch_period_s = pinch_period_s
        self.head_yaw_amp_rad = head_yaw_amp_rad
        self._tick = 0
        self._dt = 1.0 / 60.0  # 내부 카운터용 가상 dt
        self._t_override: float | None = None

    # ── 시간 제어 ──────────────────────────────────────────────────────
    def set_time(self, t: float) -> None:
        """deterministic 테스트용 — 다음 read 가 사용할 t(초) 고정."""
        self._t_override = float(t)

    def step(self, dt: float | None = None) -> None:
        """내부 가상시간 진행 (실시간 흉내)."""
        self._t_override = None
        self._tick += 1
        if dt is not None:
            self._dt = float(dt)

    @property
    def _t(self) -> float:
        if self._t_override is not None:
            return self._t_override
        return self._tick * self._dt

    # ── BridgePoseStore 호환 property ─────────────────────────────────
    @property
    def head_pose(self) -> np.ndarray:
        yaw = self.head_yaw_amp_rad * np.sin(2 * np.pi * self._t / 8.0)
        # 머리: 눈높이(local-floor 기준 y≈1.5m), 정면 -Z 응시.
        return _se3((0.0, 1.5, 0.0), rpy=(0.0, yaw, 0.0))

    def _wrist(self, side: str) -> np.ndarray:
        t = self._t
        phase = 2 * np.pi * t / self.hand_period_s
        sign = -1.0 if side == "left" else 1.0
        # 가슴 앞(local-floor): x 좌우, y 높이, z 전방(-Z).
        x = sign * 0.25 + self.hand_radius_m * np.cos(phase)
        y = 1.1 + self.hand_radius_m * np.sin(phase)
        z = -0.35
        return _se3((x, y, z), rpy=(0.0, 0.0, 0.0))

    @property
    def left_arm_pose(self) -> np.ndarray:
        return self._wrist("left")

    @property
    def right_arm_pose(self) -> np.ndarray:
        return self._wrist("right")

    def _pinch_value(self) -> float:
        # 0(닫힘)~0.08(열림) 사이 주기적 변화.
        return 0.04 * (1.0 + np.sin(2 * np.pi * self._t / self.pinch_period_s))

    @property
    def left_hand_pinchValue(self) -> float:
        return self._pinch_value()

    @property
    def right_hand_pinchValue(self) -> float:
        return self._pinch_value()

    @property
    def left_hand_pinch(self) -> bool:
        return self._pinch_value() < 0.01

    @property
    def right_hand_pinch(self) -> bool:
        return self._pinch_value() < 0.01

    @property
    def left_hand_squeezeValue(self) -> float:
        return self._pinch_value()

    @property
    def right_hand_squeezeValue(self) -> float:
        return self._pinch_value()

    @property
    def left_hand_squeeze(self) -> bool:
        return False

    @property
    def right_hand_squeeze(self) -> bool:
        return False

    def get_stats(self) -> dict:
        return {"msg_count": self._tick, "synthetic": True, "t": self._t}

    def close(self) -> None:
        return


__all__ = ["SyntheticPoseSource"]
