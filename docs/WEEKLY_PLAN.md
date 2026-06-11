# groot_teleop_dev — 주차별 개발 계획

> 상위 문서: [`DEVELOPMENT_PLAN.md`](./DEVELOPMENT_PLAN.md)
> 범례: 🤖 Claude 선개발(하드웨어 무관) · 🧑 사용자 테스트 필요 · 🔁 테스트 후 Claude 후속 개발

각 주차는 **단위(Unit)** 로 나뉘고, 단위마다 `docs/spikes/`에 spike를 남기고 git commit 한다.

---

## Phase 0 — 준비 (Week 1)

| Unit | 내용 | 구분 | 산출물 |
|---|---|---|---|
| 0.1 | 참조 repo 분석, 통합 아키텍처 확정 | 🤖 | `DEVELOPMENT_PLAN.md` |
| 0.2 | 프로젝트 스캐폴딩, `.gitignore`, 패키지 골격 | 🤖 | `groot_teleop/` 골격 |
| 0.3 | conda 환경 정의 (`envs/groot_teleop.yaml`) | 🤖 | env yaml |
| 0.4 | conda env 실제 생성 + import 스모크 | 🧑 | 환경 동작 확인 |

**완료 기준**: 프로젝트 구조 commit, env yaml 작성. (env 생성 자체는 사용자가 1회 실행)

---

## Phase 1 — 입력 어댑터 (Week 2)

| Unit | 내용 | 구분 | 산출물 |
|---|---|---|---|
| 1.1 | `teleop_dev/sender/xr_common` 브리지 vendoring + 정리 | 🤖 | `groot_teleop/input/bridge/` |
| 1.2 | `SyntheticStreamer` — 합성 pose 생성기 (헤드셋 없이) | 🤖 | `synthetic_streamer.py` + 테스트 |
| 1.3 | `GalaxyXRStreamer(BaseStreamer)` 구현 — WebXR pose → `StreamerOutput` | 🤖 | `galaxy_xr_streamer.py` |
| 1.4 | 좌표계 변환/프레임 정렬 (`xr_frame_align` 적응) + 수치 테스트 | 🤖 | `common/frames.py` + 테스트 |
| 1.5 | 단위 테스트: StreamerOutput 규격 / 좌표 변환 정확도 | 🤖 | `tests/` |
| 1.6 | 실제 헤드셋 연결 → pose 송출 진단 (`xr_pose_diag` 재활용) | 🧑 | 헤드셋 pose 확인 |

**완료 기준 (M1)**: 합성 pose로 streamer 파이프라인 단위테스트 통과, commit.
**사용자 테스트 (1.6)**: Galaxy XR/Quest3 USB-C 연결, `webxr_to_pose.html` 로 pose 송출 확인.

---

## Phase 2 — 시뮬 백엔드 브리지 (Week 3–4)

### 경로 B 우선 (이미 도커 구동 중 → 빠른 피드백)

| Unit | 내용 | 구분 | 산출물 |
|---|---|---|---|
| 2.1 | unitree_sim DDS 인터페이스 분석 (topic/메시지 schema) | 🤖 | `docs/spikes/` |
| 2.2 | XR 손목 pose → G1 상체 관절 IK (Pink/pinocchio, 오프라인) | 🤖 | `backends/unitree_dds/arm_ik.py` |
| 2.3 | DDS publisher 브리지 (IK 결과 → unitree DDS topic) | 🤖 | `backends/unitree_dds/dds_bridge.py` |
| 2.4 | 오프라인 IK 수치 검증 (목표 pose ↔ FK 재계산 오차) | 🤖 | `tests/` |
| 2.5 | 도커 sim + DDS 브리지 + 합성 pose 통합 구동 | 🧑 | sim 내 팔 동작 |
| 2.6 | (2.5 후) 손 dex_retargeting → dex3/dg5f DDS 연결 | 🔁 | 손 동작 |

### 경로 A 병행 (SONIC 전신)

| Unit | 내용 | 구분 | 산출물 |
|---|---|---|---|
| 2.7 | GR00T MuJoCo sim 설치 스크립트 conda 이식 | 🤖 | `scripts/install_groot_sim.sh` |
| 2.8 | `GalaxyXRStreamer` → GR00T ZMQ teleop 파이프라인 결선 | 🤖 | `backends/groot_mujoco/` |
| 2.9 | SONIC checkpoint 다운로드 (HF 인증) + dummy policy 대체 검증 | 🧑 | checkpoint |
| 2.10 | MuJoCo 실시간 teleop 구동 | 🧑 | sim 내 G1 동작 |

**완료 기준 (M2/M3)**: IK 오프라인 검증 통과. 사용자 테스트로 sim 팔 동작 확인.

---

## Phase 3 — 실시간 Teleop 통합 (Week 5–6)

| Unit | 내용 | 구분 | 산출물 |
|---|---|---|---|
| 3.1 | E-Stop / pause / recalibrate / speed 제어 신호 결선 | 🤖 | control_data 처리 |
| 3.2 | 워크스페이스 제한 / watchdog (안전) — teleop_dev 재활용 | 🤖 | `common/safety.py` |
| 3.3 | 지연/지터 측정 + 필터(ExpFilter) 튜닝 하니스 | 🤖 | `replay/latency.py` |
| 3.4 | 실 헤드셋 → 실시간 상체 추종 튜닝 | 🧑 | M4 |
| 3.5 | (3.4 후) 보행 명령(하체 SONIC) 통합 | 🔁 | M5 준비 |
| 3.6 | 전신 보행+조작 동시 teleop | 🧑 | M5 |

---

## Phase 4 — 정밀도 향상: VR + Vive Tracker (Week 7, 1차 성공 후)

| Unit | 내용 | 구분 |
|---|---|---|
| 4.1 | `teleop_dev` Vive sender(openvr) → `ViveStreamer(BaseStreamer)` 이식 | 🤖 |
| 4.2 | Vive Tracker 발목/허리 트래킹 → SONIC 전신 입력 매핑 | 🤖 |
| 4.3 | SteamVR 캘리브레이션 + 트래커 장착 테스트 | 🧑 |

---

## Phase 5 — 데이터 수집 & VLA (Week 8+, 선택)

| Unit | 내용 | 구분 |
|---|---|---|
| 5.1 | GR00T data_collection 파이프라인 conda 이식 | 🤖 |
| 5.2 | teleop demo 데이터 수집 (sim) | 🧑 |
| 5.3 | Isaac-GR00T N1.x fine-tune / VLA 추론 | 🧑🔁 |

---

## Phase 6 — 실제 G1 배포 (Week 9+, 최종)

| Unit | 내용 | 구분 |
|---|---|---|
| 6.1 | GR00T `gear_sonic_deploy` C++ 빌드 환경 분석 | 🤖 |
| 6.2 | 실로봇 DDS / 안전 인터록 검토 | 🤖🧑 |
| 6.3 | 실제 G1 단계적 배포 (서스펜션→보행) | 🧑 |

---

## 마일스톤 요약

| MS | 내용 | Phase | 의존 |
|---|---|---|---|
| M1 | 입력 어댑터 단위테스트 통과 | 1 | 🤖 only |
| M2 | XR→G1 IK 오프라인 검증 | 2 | 🤖 only |
| M3 | sim에서 합성 pose 팔 동작 | 2 | 🧑 |
| M4 | 실헤드셋 실시간 상체 추종 | 3 | 🧑 |
| M5 | 전신 보행+조작 teleop | 3 | 🧑 |
| M6 | 실제 G1 배포 | 6 | 🧑 |

**이번 세션 Claude 목표**: Phase 0 전체 + Phase 1 (1.1–1.5) + Phase 2 (2.1–2.4) 의 🤖 항목까지
선개발하고, 각 단위 commit + spike 작성. 🧑 항목은 `USER_TEST_HANDOFF.md`로 인계.
