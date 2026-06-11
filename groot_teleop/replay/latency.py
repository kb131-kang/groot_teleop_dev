"""오프라인 지연 측정 — streamer.get() + bridge.step() 파이프라인 latency.

헤드셋/sim/DDS 없이 (RecordingPublisher + SyntheticPoseSource) per-step
처리시간을 측정해 목표 제어주기(예: 60Hz=16.7ms) 안에 드는지 사전 검증한다.
실측 네트워크/렌더 지연은 USER_TEST(Phase 3.4)에서 별도 측정.

사용:
    python -m groot_teleop.replay.latency --steps 1000
"""

from __future__ import annotations

import argparse
import time

import numpy as np


def run(steps: int = 1000, rate: float = 60.0, scale: float = 0.5) -> dict:
    from groot_teleop.backends.unitree_dds.dds_bridge import (
        G1DDSBridge, IKTargetMapper, RecordingPublisher,
    )
    from groot_teleop.backends.unitree_dds.g1_arm_ik import IKConfig
    from groot_teleop.input.galaxy_xr_streamer import GalaxyXRStreamer
    from groot_teleop.input.synthetic_streamer import SyntheticPoseSource

    src = SyntheticPoseSource()
    streamer = GalaxyXRStreamer(pose_source=src)
    bridge = G1DDSBridge(
        publisher=RecordingPublisher(),
        mapper=IKTargetMapper(scale=scale),
        ik_config=IKConfig(max_iters=50, eps=1e-3, n_restarts=1),
    )

    dt_get = np.zeros(steps)
    dt_step = np.zeros(steps)
    period = 1.0 / rate
    for k in range(steps):
        src.step(period)
        t0 = time.perf_counter()
        out = streamer.get()
        t1 = time.perf_counter()
        bridge.step(out)
        t2 = time.perf_counter()
        dt_get[k] = (t1 - t0) * 1e3
        dt_step[k] = (t2 - t1) * 1e3

    total = dt_get + dt_step
    stats = {
        "steps": steps,
        "budget_ms": period * 1e3,
        "get_ms": _pct(dt_get),
        "ikstep_ms": _pct(dt_step),
        "total_ms": _pct(total),
        "over_budget_pct": float(100.0 * np.mean(total > period * 1e3)),
    }
    return stats


def _pct(a: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(a)),
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "max": float(np.max(a)),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--rate", type=float, default=60.0)
    args = ap.parse_args()
    s = run(args.steps, args.rate)
    print(f"제어주기 예산: {s['budget_ms']:.2f} ms @ {args.rate} Hz")
    print(f"streamer.get : mean {s['get_ms']['mean']:.3f}  p95 {s['get_ms']['p95']:.3f} ms")
    print(f"ik+publish   : mean {s['ikstep_ms']['mean']:.3f}  p95 {s['ikstep_ms']['p95']:.3f}  max {s['ikstep_ms']['max']:.3f} ms")
    print(f"total        : mean {s['total_ms']['mean']:.3f}  p95 {s['total_ms']['p95']:.3f}  max {s['total_ms']['max']:.3f} ms")
    print(f"예산 초과 프레임: {s['over_budget_pct']:.1f}%")


if __name__ == "__main__":
    main()
