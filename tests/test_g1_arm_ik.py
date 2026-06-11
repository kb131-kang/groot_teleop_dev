"""G1 팔 IK 오프라인 검증 — 헤드셋/sim 무관, FK round-trip (M2).

pinocchio + G1 URDF 필요. 둘 중 하나라도 없으면 skip (CI 안전).
"""

import numpy as np
import pytest

from groot_teleop.backends.unitree_dds import joint_layout as L

pin = pytest.importorskip("pinocchio")
from groot_teleop.backends.unitree_dds.g1_arm_ik import (  # noqa: E402
    G1ArmIK,
    IKConfig,
    default_urdf_path,
)

if not default_urdf_path().exists():
    pytest.skip("G1 URDF 없음 (external/ GR00T clone 필요)", allow_module_level=True)


@pytest.fixture(scope="module", params=["left", "right"])
def ik(request):
    return G1ArmIK(request.param, config=IKConfig(max_iters=200, eps=1e-5))


def test_reduced_model_is_7dof(ik):
    assert ik.nq == 7, f"팔 reduced model 이 7-DoF 가 아님: {ik.nq}"


def test_fk_runs(ik):
    q = np.zeros(ik.nq)
    X = ik.fk(q)
    assert X.translation.shape == (3,)


def test_ik_roundtrip_reachable_targets(ik):
    """난수 팔 자세 → FK target → IK 복원 → FK 오차 < 5mm / 1°.

    reachable 한 target 만 쓰므로 반드시 수렴해야 한다.
    """
    rng = np.random.default_rng(42)
    lo, hi = ik.model.lowerPositionLimit, ik.model.upperPositionLimit
    n_ok = 0
    N = 25
    for _ in range(N):
        # 한계 내부 80% 영역에서 난수 자세.
        q_true = lo + (0.1 + 0.8 * rng.random(ik.nq)) * (hi - lo)
        target = ik.fk(q_true)
        q_sol, err, conv = ik.solve(target, q_init=np.zeros(ik.nq))
        X = ik.fk(q_sol)
        pos_err = float(np.linalg.norm(X.translation - target.translation))
        rot_err = float(
            np.linalg.norm(pin.log3(X.rotation.T @ target.rotation))
        )
        if pos_err < 5e-3 and rot_err < np.deg2rad(1.0):
            n_ok += 1
    # 적어도 90% 는 수렴해야 (redundant 7-DoF, reachable target).
    assert n_ok >= 0.9 * N, f"round-trip 성공 {n_ok}/{N}"


def test_ik_respects_joint_limits(ik):
    rng = np.random.default_rng(7)
    target = ik.fk(rng.uniform(-0.5, 0.5, ik.nq))
    q_sol, _, _ = ik.solve(target)
    assert np.all(q_sol >= ik.model.lowerPositionLimit - 1e-9)
    assert np.all(q_sol <= ik.model.upperPositionLimit + 1e-9)


def test_joint_layout_matches_urdf(ik):
    """joint_layout 의 팔 관절명이 URDF reduced model 에 실제 존재."""
    arm_names = L.LEFT_ARM if ik.side == "left" else L.RIGHT_ARM
    for n in arm_names:
        assert n in ik.model.names, f"URDF 에 없는 관절: {n}"
