"""G1 DDS 브리지 — XR StreamerOutput → 29-DoF lowcmd publish.

데이터 흐름:
    GalaxyXRStreamer.get() → ik_data{left_wrist,right_wrist}
      → G1ArmIK(left/right) 7-DoF q ×2 = 14
      → 29-DoF q 조립 (팔 15–28, 하체 0–14 = hold)
      → Publisher.publish(...)   (rt/lowcmd, unitree_hg.LowCmd_)

publisher 를 추상화해 헤드셋/DDS 없이 단위테스트 가능:
    RecordingPublisher  — 메시지 캡처(테스트)
    UnitreeDDSPublisher — 실제 rt/lowcmd publish (unitree_sdk2py lazy import)

⚠️ XR 손목 pose 는 headset-relative frame 이다. 로봇 팔 base(pelvis) frame
으로의 매핑(calibration: 위치 offset + 스케일)은 ``IKTargetMapper`` 로 분리했고,
기본값은 합리적 추정치이다. 실제 정합은 USER_TEST(Phase 3 calibration)에서 튜닝.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol

import numpy as np

from groot_teleop.backends.unitree_dds import joint_layout as L
from groot_teleop.backends.unitree_dds.g1_arm_ik import (
    G1ArmIK,
    IKConfig,
    se3_from_matrix,
)


# ── publisher 추상화 ────────────────────────────────────────────────────
class Publisher(Protocol):
    def publish(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        torques: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
    ) -> None: ...


@dataclass
class RecordingPublisher:
    """DDS 없이 publish 를 캡처 (테스트/오프라인)."""

    history: list = field(default_factory=list)
    last: Optional[dict] = None

    def publish(self, positions, velocities, torques, kp, kd) -> None:
        msg = {
            "positions": np.asarray(positions, dtype=np.float64).copy(),
            "velocities": np.asarray(velocities, dtype=np.float64).copy(),
            "torques": np.asarray(torques, dtype=np.float64).copy(),
            "kp": np.asarray(kp, dtype=np.float64).copy(),
            "kd": np.asarray(kd, dtype=np.float64).copy(),
        }
        self.last = msg
        self.history.append(msg)


class UnitreeDDSPublisher:
    """실제 rt/lowcmd publish — unitree_sdk2py 필요 (lazy import).

    sim/실로봇이 같은 네트워크에서 rt/lowcmd 를 구독한다. 사용 전
    ``ChannelFactoryInitialize(0, iface)`` 가 호출돼 있어야 한다 (run 스크립트
    또는 외부에서). USER_TEST 단계에서 도커 sim 과 함께 검증.
    """

    def __init__(self, topic: str = "rt/lowcmd", init_channel: bool = False,
                 domain_id: int = 0, iface: str = ""):
        from unitree_sdk2py.core.channel import ChannelPublisher  # lazy
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        if init_channel:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            ChannelFactoryInitialize(domain_id, iface)

        self._LowCmd_ = LowCmd_
        self._make_cmd = unitree_hg_msg_dds__LowCmd_
        self._crc = CRC()
        self.pub = ChannelPublisher(topic, LowCmd_)
        self.pub.Init()

    def publish(self, positions, velocities, torques, kp, kd) -> None:
        cmd = self._make_cmd()
        for i in range(L.NUM_JOINTS):
            m = cmd.motor_cmd[i]
            m.mode = 1
            m.q = float(positions[i])
            m.dq = float(velocities[i])
            m.tau = float(torques[i])
            m.kp = float(kp[i])
            m.kd = float(kd[i])
        cmd.crc = self._crc.Crc(cmd)
        self.pub.Write(cmd)


# ── XR pose → 로봇 IK target 매핑 ──────────────────────────────────────
@dataclass
class IKTargetMapper:
    """headset-relative 손목 4×4 → 로봇 팔 base frame IK target.

    target_p = origin + scale * wrist_p_relative ; target_R = wrist_R
    origin 은 사용자가 팔을 내린 중립에서 손이 위치할 로봇 frame 지점.
    scale 로 사람↔로봇 팔길이 차이 보정. 실제 값은 calibration 으로 결정.
    """

    left_origin: np.ndarray = field(
        default_factory=lambda: np.array([0.25, 0.20, 0.10])
    )
    right_origin: np.ndarray = field(
        default_factory=lambda: np.array([0.25, -0.20, 0.10])
    )
    scale: float = 1.0

    def map(self, wrist_T: np.ndarray, side: str) -> np.ndarray:
        origin = self.left_origin if side == "left" else self.right_origin
        T = np.eye(4)
        T[:3, :3] = wrist_T[:3, :3]
        T[:3, 3] = origin + self.scale * wrist_T[:3, 3]
        return T


# ── 브리지 ──────────────────────────────────────────────────────────────
class G1DDSBridge:
    def __init__(
        self,
        publisher: Publisher,
        mapper: Optional[IKTargetMapper] = None,
        ik_config: Optional[IKConfig] = None,
        hold_q: Optional[np.ndarray] = None,
        urdf_path=None,
    ):
        self.pub = publisher
        self.mapper = mapper or IKTargetMapper()
        self.left_ik = G1ArmIK("left", urdf_path=urdf_path, config=ik_config)
        self.right_ik = G1ArmIK("right", urdf_path=urdf_path, config=ik_config)
        # 하체+허리(0–14) 유지 자세. 기본 0 (서있는 중립). SONIC 연결 전까지 hold.
        self.hold_q = (
            np.asarray(hold_q, dtype=np.float64)
            if hold_q is not None
            else np.zeros(L.NUM_JOINTS)
        )
        self._q_left = None   # warm-start
        self._q_right = None
        self.kp = np.zeros(L.NUM_JOINTS)
        self.kd = np.zeros(L.NUM_JOINTS)
        self.kp[L.ARM_INDICES] = L.DEFAULT_ARM_KP
        self.kd[L.ARM_INDICES] = L.DEFAULT_ARM_KD

    def step(self, streamer_output) -> dict:
        """StreamerOutput 1 frame → lowcmd publish. 디버그 정보 dict 반환."""
        ik = streamer_output.ik_data
        left_T = self.mapper.map(np.asarray(ik["left_wrist"]), "left")
        right_T = self.mapper.map(np.asarray(ik["right_wrist"]), "right")

        q_l, err_l, conv_l = self.left_ik.solve(
            se3_from_matrix(left_T), q_init=self._q_left
        )
        q_r, err_r, conv_r = self.right_ik.solve(
            se3_from_matrix(right_T), q_init=self._q_right
        )
        self._q_left, self._q_right = q_l, q_r

        positions = self.hold_q.copy()
        positions[L.LEFT_ARM_INDICES] = q_l
        positions[L.RIGHT_ARM_INDICES] = q_r
        velocities = np.zeros(L.NUM_JOINTS)
        torques = np.zeros(L.NUM_JOINTS)

        self.pub.publish(positions, velocities, torques, self.kp, self.kd)
        return {
            "positions": positions,
            "ik_err": (err_l, err_r),
            "ik_converged": (conv_l, conv_r),
        }


__all__ = [
    "Publisher", "RecordingPublisher", "UnitreeDDSPublisher",
    "IKTargetMapper", "G1DDSBridge",
]
