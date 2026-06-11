# Spike 02 — unitree_sim DDS 백엔드 브리지 (경로 B)

**Phase 2 / Unit 2.1–2.4** · 상태: ✅ 완료 (Claude 단독, 헤드셋/sim 무관)

## 목표
XR 손목 pose → G1 14-DoF 팔 IK → unitree `rt/lowcmd` DDS publish 경로를
구현하고, 헤드셋·sim·DDS 없이 오프라인 검증한다.

## DDS 인터페이스 분석 (2.1)
unitree_sim_isaaclab(`action_provider_dds.py`, `g1_robot_dds.py`)는 실제 G1 과
동일하게 다음을 구독:
- `rt/lowcmd` (`unitree_hg.LowCmd_`) — 29개 motor_cmd(q/dq/tau/kp/kd).
- 팔 = 14 관절: lowcmd index **15–28** (좌 15–21, 우 22–28). 하체+허리 = 0–14.
- 손: `rt/dex3/{left,right}/cmd`, `rt/dex1/*`, `rt/dg5f/cmd`(Tesollo, fork 추가).

## 구현
| 파일 | 역할 | 검증 |
|---|---|---|
| `backends/unitree_dds/joint_layout.py` | G1 29-DoF 이름/인덱스 상수 | 인덱스/중복 단위테스트 |
| `backends/unitree_dds/g1_arm_ik.py` | pinocchio per-arm DLS IK (7-DoF, reduced model) | FK round-trip |
| `backends/unitree_dds/dds_bridge.py` | 29-DoF lowcmd 조립 + pluggable publisher | RecordingPublisher 테스트 |
| `scripts/run_unitree_dds_teleop.py` | 실행 스크립트 (dry-run/synthetic/reachable-demo/real) | dry-run 스모크 |
| `replay/latency.py` | 오프라인 지연 측정 | 60Hz 예산 검증 |

## 핵심 설계
- **pluggable publisher**: `RecordingPublisher`(테스트) ↔ `UnitreeDDSPublisher`
  (실 DDS, unitree_sdk2py lazy import) → DDS 없이 전 파이프라인 테스트.
- **reduced-model IK**: 팔 7개 외 전부 lock → 7-DoF 만 최적화. DLS + step clamp
  + deterministic random restart 로 local minima 회피. 실시간은 warm-start.
- **IKTargetMapper**: XR(headset-relative) → 로봇 팔 base frame 매핑 분리.
  calibration(origin/scale)은 USER_TEST 사안으로 명시 격리.

## 검증 결과 (pinocchio 2.7 + G1 URDF, 헤드셋 없이)
```
pytest tests/  → 30 passed
  - IK round-trip: reachable target 90%+ 수렴 (<5mm/1°)
  - bridge: 29-DoF shape, 하체 hold, 팔 구동/추종, reachable 궤적 warm-start 수렴
python scripts/run_unitree_dds_teleop.py --dry-run --reachable-demo  → 240 msg, conv=True
python -m groot_teleop.replay.latency  → total mean 4.8ms (60Hz 예산 16.7ms, 초과 0%)
```

## 발견 / 한계 (정직한 기록)
- **synthetic 손 workspace ↔ G1 팔 reachable set 미정합**: 기본 mapper 로는
  IK 가 best-effort(미수렴) → 팔은 움직이나 정합 안 됨. **calibration 이 필요**
  하며 이는 USER_TEST(Phase 3) 사안. `--reachable-demo` 는 FK 로 reachable
  궤적을 만들어 정합 없이도 깔끔한 sim 팔 동작을 보여준다(M3 첫 테스트용).
- 손(dex3/dg5f) DDS 연결은 미구현 — Phase 2.6 (사용자 sim 팔 동작 확인 후).

## 다음 (🧑 USER_TEST → `USER_TEST_HANDOFF.md`)
1. 도커 sim 구동 + `run_unitree_dds_teleop.py --reachable-demo --iface ...` →
   sim 에서 G1 팔이 부드럽게 움직이는지 확인 (M3).
2. DDS iface/도메인/`cyclonedds.xml` 정합 확인 (host↔컨테이너).
3. 그 후: IKTargetMapper calibration → 실제 XR 손 동작 추종(M4).
