# 사용자 테스트 인계 (USER TEST HANDOFF)

> Claude 가 하드웨어 없이 선개발한 부분이 끝났다. 이 문서는 **사용자가 직접
> 테스트해야 하는 부분**과, 각 테스트 통과 후 **이어서 개발할 부분**을 순서대로
> 정리한다. 게이트(gate) 순서대로 진행하면 된다.

현재까지 Claude 가 완료한 것 (헤드셋/sim/실로봇 무관, 전부 자동 검증됨):
- 통합 아키텍처 + 주차별 계획 (`DEVELOPMENT_PLAN.md`, `WEEKLY_PLAN.md`)
- XR 입력 어댑터 `GalaxyXRStreamer` (PICO drop-in) + WebXR 브리지
- 경로 B DDS 브리지: XR→G1 팔 IK→`rt/lowcmd` (pinocchio, 안전 clamp, relative/recalibrate, watchdog)
- 단위테스트 **33 passed**, 지연 측정(60Hz 예산 내), dry-run 스모크

---

## 게이트 0 — conda 환경 생성 (필수 선행)

```bash
cd /media/ys/data2/groot_teleop_dev
conda env create -f envs/groot_teleop.yaml      # groot_teleop env 생성
conda activate groot_teleop
pip install -e .

# 검증
pytest tests/ -v                                 # 33 passed 기대
python -m groot_teleop.input.bridge.bridge_pose_store --selftest   # PASS 기대
python -m groot_teleop.replay.latency            # 60Hz 예산 내 확인
```
**통과 기준**: 테스트 33 통과, selftest PASS.
**문제 시**: `pin`/`dex_retargeting` 설치 실패면 `envs/groot_teleop.yaml` 의 pip
섹션을 conda-forge 채널로 조정(예: `conda install -c conda-forge pinocchio`).
- 참고: 빠른 검증은 기존 `teleop_operator` env 로도 가능(Claude 가 그렇게 검증함).

> ⚠️ 이후 모든 명령은 `conda activate groot_teleop` 상태에서 실행.

---

## 게이트 1 — XR 헤드셋 pose 송출 확인 (입력 계층, M-입력)

**목적**: Galaxy XR / Quest 3 가 손/머리 pose 를 조종 PC 로 보내는지.

```bash
# 1) 헤드셋 USB-C 연결 후
adb reverse tcp:8013 tcp:8013
# 2) 브리지 server 기동
python -m groot_teleop.input.bridge.bridge_pose_store
# 3) 헤드셋 Chrome 에서 http://localhost:8013/ 접속 → "Enter VR/AR" → 손 움직이기
#    터미널에 head/hand msg 카운트가 증가하면 성공.
```
**통과 기준**: `msgs=`, `hand=` 카운트가 손 움직임에 따라 증가.
**통과 후 Claude 개발**:
- 실제 pose 로 `GalaxyXRStreamer.get()` 진단 스크립트(`scripts/xr_diag.py`) 작성.
- WebXR 손목 ↔ PICO 손목 **회전 offset** 측정값 받아 `common/frames.py` 반영.

---

## 게이트 2 — 헤드셋 없이 sim 팔 동작 (경로 B, M3) ⭐ 가장 먼저 추천

**목적**: DDS→Isaac sim 결선이 되는지 (헤드셋 불필요, 이미 구동 중인 도커 활용).

```bash
# 1) 도커 sim 구동 (unitree_sim_isaaclab, rt/lowcmd 구독 task)
#    예: /media/ys/data2/datasets/unitree_sim_isaaclab 에서
#    ./run_sim.sh --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
#                 --enable_dex3_dds --robot_type g129 --device cuda:0 --enable_cameras
#
# 2) (호스트 또는 컨테이너) reachable-demo 송신 — 헤드셋 없이 깔끔한 팔 동작
python scripts/run_unitree_dds_teleop.py --reachable-demo --iface <DDS_IFACE>
```
**확인사항**:
- sim 안에서 G1 양팔이 부드럽게 흔들리는가?
- DDS iface/도메인/`cyclonedds.xml` 가 호스트↔컨테이너에서 정합하는가?
  (sim 과 송신측이 같은 DDS domain/네트워크여야 함. 컨테이너 host-net 또는
  공유 iface 확인.)

**통과 기준**: sim 의 G1 팔이 송신 궤적을 따라 움직인다.
**실패 디버그**: 먼저 `--dry-run --reachable-demo` 로 송신측 정상 확인(이미 검증됨)
→ 그래도 sim 이 안 움직이면 DDS 네트워크/도메인 문제. `--topic`, `--domain-id`,
`--iface` 점검. 컨테이너 안에서 직접 실행하는 편이 네트워크 단순.
**통과 후 Claude 개발**:
- 게이트 1 통과 시: `--synthetic` 대신 실제 XR 입력으로 전환 + **calibration**
  (`IKTargetMapper` origin/scale) 도구 작성.
- `rt/lowstate` 구독 → 폐루프 robot_origin + warm-start (spike 03 후속).

---

## 게이트 3 — 실제 XR 손 동작으로 sim 팔 추종 (M4)

**전제**: 게이트 1, 2 통과.
```bash
# 헤드셋 연결 + adb reverse 후
python scripts/run_unitree_dds_teleop.py --relative --scale 0.6 --iface <DDS_IFACE>
# --relative: 손을 편한 위치에 두고 시작(첫 프레임 자동 recalibrate).
```
**확인사항**: 손을 움직이면 sim G1 팔이 같은 방향으로 따라오는가? 좌우/상하/전후
방향이 맞는가(거울 반전 없는가)?
**통과 기준**: 손↔팔 방향 일치, 점프 없이 부드러운 추종.
**통과 후 Claude 개발**:
- 방향 불일치 시 `frames` 회전 offset / mirror 보정.
- 손가락(pinch)→ dex3/dg5f 그리퍼 DDS 연결 (Phase 2.6).
- 지연/지터 실측 → ExpFilter 튜닝.

---

## 게이트 4 — 경로 A: GR00T SONIC 전신 teleop (M5, MuJoCo)

**목적**: 보행+조작 동시 전신 제어.
```bash
# 1) GR00T MuJoCo 환경 설치 (uv→conda 이식 또는 GR00T install_scripts)
#    bash external/GR00T-WholeBodyControl/install_scripts/install_mujoco_sim.sh
# 2) SONIC checkpoint 다운로드 (HuggingFace 인증 필요)
#    python external/GR00T-WholeBodyControl/download_from_hf.py
# 3) GR00T teleop_streamer 에 galaxy_xr 분기 1줄 추가
#    → groot_teleop/backends/groot_mujoco/INTEGRATION.md 참조
# 4) GR00T ZMQ teleop 실행 (PICO 대신 galaxy_xr device)
```
**확인사항**: 헤드셋으로 G1 상체 조작 + (보행 입력 결선 후) 이동.
**통과 후 Claude 개발**:
- 보행 명령 입력 결선(thumbstick 없음 → Vive 발목 or 제스처, Phase 3.5).
- base height / activation 제스처 매핑.

---

## 게이트 5 — VR + Vive Tracker 정밀 전신 (Phase 4, 1차 성공 후)

**전제**: 게이트 3/4 성공.
- `teleop_dev/sender/arm/vive_*` → `ViveStreamer(BaseStreamer)` 이식(Claude 선개발 가능).
- SteamVR 캘리브레이션 + 발목/허리 트래커 장착 (🧑 사용자).

---

## 게이트 6 — 실제 G1 로봇 배포 (최종, 안전 검토 필수)

- GR00T `gear_sonic_deploy` C++ 빌드 (Claude 분석 가능, 빌드/실행 🧑).
- 서스펜션 거치 → 보행 순으로 단계적. E-Stop 하드웨어 인터록 필수.

---

## 요약 — 테스트 순서 추천

```
게이트 0 (env)  →  게이트 2 (헤드셋 없이 sim 팔, 가장 빠른 피드백) ─┐
                                                                  ├→ 게이트 3 (실 XR→sim)
게이트 1 (헤드셋 pose) ────────────────────────────────────────────┘
                                                                  →  게이트 4 (GR00T 전신)
                                                                  →  게이트 5 (Vive) → 게이트 6 (실로봇)
```

각 게이트의 "통과 후 Claude 개발" 항목은 사용자가 테스트 결과(특히 방향/정합/DDS
네트워크 정보)를 알려주면 Claude 가 이어서 자동 개발한다.
