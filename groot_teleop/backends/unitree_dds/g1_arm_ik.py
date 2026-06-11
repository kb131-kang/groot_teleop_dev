"""G1 팔 IK — 손목/손바닥 target pose → 7-DoF 팔 관절각 (pinocchio).

target(palm) 6-DoF pose 를 7-DoF 팔로 추종 (1 redundancy). damped
least-squares(Levenberg-Marquardt) 로 per-arm 풀이. 비-팔 관절은 pinocchio
``buildReducedModel`` 로 잠가 reduced model 에서 7-DoF 만 최적화한다.

오프라인 검증 (헤드셋/sim 무관):
    난수 팔 자세 q → FK 로 palm target 생성 → IK 로 q' 복원 → FK(q') 오차 확인
    (round-trip). ``tests/test_g1_arm_ik.py`` 참조.

의존: pinocchio(pin) + g1_29dof_with_hand.urdf. teleop_operator env 에 pin 2.7
설치 확인됨.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from groot_teleop.backends.unitree_dds import joint_layout as L

# URDF 기본 경로 — external/ GR00T 자산. 환경변수로 override 가능.
_DEFAULT_URDF = (
    Path(__file__).resolve().parents[3]
    / "external/GR00T-WholeBodyControl/gear_sonic/data/robots/g1/g1_29dof_with_hand.urdf"
)


def default_urdf_path() -> Path:
    return Path(os.environ.get("G1_URDF", str(_DEFAULT_URDF)))


@dataclass
class IKConfig:
    max_iters: int = 100
    eps: float = 1e-4          # 수렴 위치/자세 오차 (m, rad 합)
    damping: float = 1e-6      # DLS 감쇠
    dt: float = 1.0            # 적분 step scale
    pos_only: bool = False     # True 면 위치만(자세 무시) — 디버그용
    max_step: float = 0.2      # 반복당 |dq| clamp (rad) — 발산 방지
    n_restarts: int = 4        # 수렴 실패 시 random seed 재시도 횟수
    restart_seed: int = 0      # 재시작 난수 seed (deterministic)


class G1ArmIK:
    """단일 팔(left 또는 right) IK 솔버."""

    def __init__(
        self,
        side: str,
        urdf_path: Optional[Path] = None,
        config: Optional[IKConfig] = None,
    ):
        import pinocchio as pin  # lazy — pin 없는 환경에서 import 만으로 죽지 않게

        assert side in ("left", "right"), side
        self.side = side
        self.pin = pin
        self.cfg = config or IKConfig()
        urdf_path = Path(urdf_path or default_urdf_path())
        if not urdf_path.exists():
            raise FileNotFoundError(
                f"G1 URDF 없음: {urdf_path}. external/ 에 GR00T clone 필요 "
                f"(또는 G1_URDF 환경변수 지정)."
            )

        full = pin.buildModelFromUrdf(str(urdf_path))

        arm_names = L.LEFT_ARM if side == "left" else L.RIGHT_ARM
        self.arm_names = arm_names
        ee_name = L.LEFT_EE_FRAME if side == "left" else L.RIGHT_EE_FRAME

        # 팔 7개를 제외한 모든 관절을 reference 자세에서 잠금.
        keep = set(arm_names)
        lock_ids = [
            full.getJointId(n)
            for n in full.names
            if n != "universe" and n not in keep
        ]
        q_ref = pin.neutral(full)
        self.model = pin.buildReducedModel(full, lock_ids, q_ref)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId(ee_name)
        self.nq = self.model.nq  # = 7
        # reduced model 의 관절 순서 (q 인덱스 → 관절명).
        self.reduced_joint_order = [
            n for n in self.model.names if n in keep
        ]

    # ── FK ─────────────────────────────────────────────────────────────
    def fk(self, q: np.ndarray):
        pin = self.pin
        q = np.asarray(q, dtype=np.float64)
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.ee_id].copy()  # pin.SE3

    # ── IK ─────────────────────────────────────────────────────────────
    def _dls_once(self, target_SE3, q):
        """단일 seed 에서 DLS 반복. (q, err_norm, converged) 반환."""
        pin = self.pin
        q = np.array(q, dtype=np.float64)
        err_norm = np.inf
        for _ in range(self.cfg.max_iters):
            pin.framesForwardKinematics(self.model, self.data, q)
            oMf = self.data.oMf[self.ee_id]
            err = pin.log6(oMf.actInv(target_SE3)).vector  # 6D twist
            if self.cfg.pos_only:
                err[3:] = 0.0
            err_norm = float(np.linalg.norm(err))
            if err_norm < self.cfg.eps:
                return q, err_norm, True
            J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_id)
            if self.cfg.pos_only:
                J, e = J[:3, :], err[:3]
            else:
                e = err
            JJt = J @ J.T
            JJt[np.diag_indices_from(JJt)] += self.cfg.damping
            dq = J.T @ np.linalg.solve(JJt, e)
            # 반복당 step clamp — 먼 target 에서 발산/overshoot 방지.
            step = self.cfg.dt * dq
            n = np.linalg.norm(step)
            if n > self.cfg.max_step:
                step *= self.cfg.max_step / n
            q = pin.integrate(self.model, q, step)
            q = np.clip(
                q, self.model.lowerPositionLimit, self.model.upperPositionLimit
            )
        return q, err_norm, False

    def solve(
        self,
        target_SE3,
        q_init: Optional[np.ndarray] = None,
    ) -> tuple[np.ndarray, float, bool]:
        """target(pin.SE3) → 팔 7-DoF q. (q, final_err, converged) 반환.

        1차 seed(q_init 또는 neutral)에서 DLS. 실패 시 관절한계 내 random
        seed 로 ``n_restarts`` 회 재시도하고 최선의 해를 반환 (redundant
        7-DoF 의 local minima 회피). seed 는 deterministic.

        실시간 teleop 에서는 직전 프레임 q 를 q_init 으로 넘기면 보통 1차에
        수렴하므로 restart 비용이 들지 않는다 (warm start).
        """
        pin = self.pin
        seed = (
            np.array(q_init, dtype=np.float64)
            if q_init is not None
            else pin.neutral(self.model)
        )
        best_q, best_err, conv = self._dls_once(target_SE3, seed)
        if conv:
            return best_q, best_err, True

        rng = np.random.default_rng(self.cfg.restart_seed)
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        for _ in range(self.cfg.n_restarts):
            q0 = lo + rng.random(self.nq) * (hi - lo)
            q, err, c = self._dls_once(target_SE3, q0)
            if err < best_err:
                best_q, best_err, conv = q, err, c
            if conv:
                break
        return best_q, best_err, conv

    def clamp_to_limits(self, q: np.ndarray) -> np.ndarray:
        return np.clip(
            np.asarray(q, dtype=np.float64),
            self.model.lowerPositionLimit,
            self.model.upperPositionLimit,
        )


def make_se3(R: np.ndarray, p: np.ndarray):
    """(R 3×3, p 3) → pin.SE3. pin 미설치 환경에서도 import 가능하도록 lazy."""
    import pinocchio as pin

    return pin.SE3(np.asarray(R, dtype=np.float64), np.asarray(p, dtype=np.float64))


def se3_from_matrix(T: np.ndarray):
    import pinocchio as pin

    T = np.asarray(T, dtype=np.float64)
    return pin.SE3(T[:3, :3].copy(), T[:3, 3].copy())


__all__ = ["G1ArmIK", "IKConfig", "make_se3", "se3_from_matrix", "default_urdf_path"]
