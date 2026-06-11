#!/usr/bin/env python3
"""경로 B teleop 실행 — XR(또는 합성) → G1 팔 IK → unitree rt/lowcmd DDS.

unitree_sim_isaaclab(도커, 이미 구동 중) 또는 실제 G1 이 rt/lowcmd 를 구독한다.

모드
----
--dry-run     DDS 없이 RecordingPublisher 로 파이프라인만 검증 (헤드셋·sim 무관).
--synthetic   헤드셋 없이 SyntheticPoseSource 로 sim 에 합성 팔 동작 publish.
              → **헤드셋 없이 sim 팔 동작을 보는 첫 USER_TEST (M3) 에 사용.**
(기본)        실제 Galaxy XR/Quest 3 (BridgePoseStore ws) 입력.

예시
----
  # 0) 완전 오프라인 (CI/검증)
  python scripts/run_unitree_dds_teleop.py --dry-run --steps 100

  # 1) 헤드셋 없이 sim 으로 합성 동작 송신 (sim 구동 후)
  python scripts/run_unitree_dds_teleop.py --synthetic --iface lo

  # 2) 실제 헤드셋 (adb reverse tcp:8013 후 Chrome 에서 http://localhost:8013)
  python scripts/run_unitree_dds_teleop.py --iface lo
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# editable install 없이 실행 가능하도록 repo root 를 path 에 추가.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from groot_teleop.backends.unitree_dds.dds_bridge import (
    G1DDSBridge,
    IKTargetMapper,
    RecordingPublisher,
)
from groot_teleop.backends.unitree_dds.g1_arm_ik import IKConfig
from groot_teleop.input.galaxy_xr_streamer import GalaxyXRStreamer


def _sleep_remainder(t0: float, period: float) -> None:
    dt = time.perf_counter() - t0
    if dt < period:
        time.sleep(period - dt)


def build_streamer(args):
    if args.synthetic or args.dry_run:
        from groot_teleop.input.synthetic_streamer import SyntheticPoseSource

        return GalaxyXRStreamer(pose_source=SyntheticPoseSource())
    # 실제 헤드셋: BridgePoseStore ws server 자동 기동.
    return GalaxyXRStreamer(bridge_port=args.bridge_port)


def build_publisher(args):
    if args.dry_run:
        return RecordingPublisher()
    from groot_teleop.backends.unitree_dds.dds_bridge import UnitreeDDSPublisher

    return UnitreeDDSPublisher(
        topic=args.topic, init_channel=True, domain_id=args.domain_id,
        iface=args.iface,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="DDS 없이 검증")
    ap.add_argument("--synthetic", action="store_true", help="헤드셋 없이 합성 pose")
    ap.add_argument("--reachable-demo", action="store_true",
                    help="헤드셋 없이 reachable 팔 궤적(FK 생성)을 sim 에 송신 — "
                         "calibration 전에도 깔끔한 팔 동작 확인용 (M3)")
    ap.add_argument("--iface", default="", help="DDS network interface (예: lo, eth0)")
    ap.add_argument("--domain-id", type=int, default=0)
    ap.add_argument("--topic", default="rt/lowcmd")
    ap.add_argument("--bridge-port", type=int, default=None)
    ap.add_argument("--rate", type=float, default=60.0, help="control Hz")
    ap.add_argument("--steps", type=int, default=0, help="0=무한")
    ap.add_argument("--scale", type=float, default=0.5, help="XR→robot 위치 스케일")
    ap.add_argument("--relative", action="store_true",
                    help="relative-motion 모드 (첫 프레임 자동 recalibrate)")
    ap.add_argument("--watchdog-timeout", type=float, default=0.2,
                    help="pose stale timeout(s) — 초과 시 hold (실 헤드셋 모드)")
    args = ap.parse_args()

    streamer = build_streamer(args)
    streamer.start_streaming()
    publisher = build_publisher(args)
    bridge = G1DDSBridge(
        publisher=publisher,
        mapper=IKTargetMapper(scale=args.scale),
        ik_config=IKConfig(max_iters=50, eps=1e-3, n_restarts=1),
        relative=args.relative,
    )

    # 실 헤드셋 모드: pose freshness watchdog (stale → hold).
    watchdog = None
    if not (args.dry_run or args.synthetic or args.reachable_demo):
        from groot_teleop.input.bridge.watchdog import StoreWatchdog

        if hasattr(streamer, "source") and hasattr(streamer.source, "get_stats"):
            watchdog = StoreWatchdog(streamer.source, timeout_s=args.watchdog_timeout)

    mode = "dry-run" if args.dry_run else ("synthetic" if args.synthetic else "Galaxy-XR")
    print(f"[run] mode={mode} rate={args.rate}Hz topic={args.topic} iface='{args.iface}'")
    print("[run] Ctrl+C 로 종료")

    # reachable-demo: 팔 FK 로 도달 가능한 손목 궤적을 직접 생성 → 깔끔한 동작.
    demo = None
    if args.reachable_demo:
        from types import SimpleNamespace
        lik, rik = bridge.left_ik, bridge.right_ik
        bridge.mapper.left_origin[:] = 0.0
        bridge.mapper.right_origin[:] = 0.0
        bridge.mapper.scale = 1.0

        def demo(step_k):
            ql = 0.3 * np.sin(0.05 * step_k + np.arange(7))
            qr = 0.3 * np.sin(0.05 * step_k + np.arange(7) + 1.0)
            return SimpleNamespace(ik_data={
                "left_wrist": lik.fk(lik.clamp_to_limits(ql)).homogeneous,
                "right_wrist": rik.fk(rik.clamp_to_limits(qr)).homogeneous,
            })

    period = 1.0 / args.rate
    k = 0
    src = getattr(streamer, "source", None)
    try:
        while args.steps == 0 or k < args.steps:
            t0 = time.perf_counter()
            if hasattr(src, "step"):  # synthetic: 가상시간 진행
                src.step(period)
            # watchdog: pose 가 stale 이면 hold (stale target 송신 차단).
            if watchdog is not None and not watchdog.fresh():
                bridge.hold()
                k += 1
                _sleep_remainder(t0, period)
                continue

            out = demo(k) if demo is not None else streamer.get()
            # relative 모드: 첫 유효 프레임에서 자동 recalibrate.
            if args.relative and k == 0:
                bridge.step(out)            # _q 초기화
                bridge.recalibrate(out)
            info = bridge.step(out)
            if k % int(args.rate) == 0:
                el, er = info["ik_err"]
                cl, cr = info["ik_converged"]
                print(f"[{k:6d}] ik_err L={el:.4f} R={er:.4f} conv={cl,cr}")
            k += 1
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[run] stopped.")
    finally:
        streamer.stop_streaming()
        if isinstance(publisher, RecordingPublisher):
            print(f"[run] dry-run published {len(publisher.history)} lowcmd msgs")
            if publisher.last is not None:
                arms = publisher.last['positions'][15:29]
                print(f"[run] last arm q (14): {np.round(arms, 3)}")


if __name__ == "__main__":
    main()
