# groot_teleop_dev — 휴머노이드 원격조종 시스템 개발 계획서

> **방법론**: NVlabs [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) (GEAR-SONIC Whole-Body Control)
> **대상 로봇**: Unitree G1 (29-DoF) — 시뮬레이션 우선 → 실제 로봇
> **입력 장비**: Galaxy XR / Meta Quest 3 (1차) → VR + Vive Tracker (2차)
> **작성일**: 2026-06-11

---

## 1. 프로젝트 개요

### 1.1 목표
사람의 전신 움직임을 XR 헤드셋으로 캡처하여 휴머노이드 로봇(G1)을 실시간 원격조종한다.
GR00T-WBC의 **SONIC whole-body control**(하체 RL 보행 + 상체 IK 조작)을 제어 백본으로 사용하고,
입력 장비 계층은 자체 개발한 `teleop_dev`의 **WebXR 브리지**를 재활용한다.

### 1.2 핵심 설계 결정 (사용자 요구사항 반영)
| 항목 | 결정 | 근거 |
|---|---|---|
| 제어 방법론 | GR00T-WBC (SONIC) | 검증된 G1 전신 제어 + teleop 스택 제공 |
| 입력 장비 (1차) | Galaxy XR / Quest 3 | 보유 장비, WebXR 브리지 기존 개발됨 (`teleop_dev`) |
| 입력 장비 (2차) | VR + Vive Tracker | 전신 트래킹 정밀도 향상 (성공 시 추가) |
| 로봇 (1차) | 시뮬레이션 | 안전, 빠른 반복. 2개 경로 비교 검토 |
| 로봇 (2차) | 실제 G1 | 시뮬 검증 후 |
| 환경 관리 | **conda** (uv 아님) | 로컬 PC 충돌 최소화 (GR00T 기본은 uv venv → conda로 이식) |
| 개발 방식 | Claude 선개발 → 사용자 테스트 | 하드웨어 무관 부분 최대 자동 개발 |

---

## 2. 시스템 아키텍처

### 2.1 통합 데이터 흐름

```
┌─────────────────── Operator PC (조종 PC, conda) ───────────────────┐
│                                                                    │
│  [입력 계층 — teleop_dev 재활용]                                    │
│   Galaxy XR / Quest 3  ──USB-C(adb reverse)──┐                      │
│     Chrome WebXR (webxr_to_pose.html)        │                      │
│                                              ▼                      │
│   ws bridge (port 8013) ──→ BridgePoseStore (head/L/R wrist pose)   │
│   [2차] Vive Tracker (openvr) ──────────────→ (동일 store 인터페이스)│
│                                              │                      │
│  [어댑터 계층 — 본 프로젝트 신규 개발]          ▼                      │
│   GalaxyXRStreamer(BaseStreamer)  ──→ StreamerOutput                │
│     · ik_data: 손목 4×4 pose (L/R), head pose, base height          │
│     · control_data: estop / pause / recalibrate / speed             │
│     · teleop_data: policy toggle / data-collect toggle              │
│                                              │                      │
└──────────────────────────────────────────────┼────────────────────┘
                                                │
              ┌─────────────────────────────────┴──────────────┐
              ▼ (경로 A)                                         ▼ (경로 B)
┌──────────────────────────────┐          ┌──────────────────────────────────┐
│  GR00T-WBC MuJoCo + ZMQ      │          │  unitree_sim_isaaclab (DDS)        │
│  · body IK solver (상체)      │          │  · Isaac Lab, 실제 G1과 동일 DDS    │
│  · SONIC WBC policy (하체)    │          │  · XR pose → arm IK → DDS publish  │
│  · sim2mujoco 시각화          │          │  · 이미 도커 컨테이너 구동 중        │
└──────────────────────────────┘          └──────────────────────────────────┘
              │                                         │
              └──────────────── (2차) 실제 G1 ──────────┘
                         GR00T C++ deploy stack / DDS
```

### 2.2 계층별 역할 및 출처

| 계층 | 컴포넌트 | 출처 | 본 프로젝트 작업 |
|---|---|---|---|
| 입력 | WebXR → pose 브리지 | `teleop_dev/sender/xr_common` | **재활용** (vendoring) |
| 입력 | XR 프레임 정렬/recalibrate | `teleop_dev/sender/arm/xr_frame_align.py` | **재활용 + 적응** |
| 입력 | Vive Tracker | `teleop_dev/sender/arm/vive_*.py` | **재활용** (2차) |
| 어댑터 | `GalaxyXRStreamer` | **신규** | GR00T `BaseStreamer` 구현 |
| 리타게팅 | 상체 body IK | `GR00T decoupled_wbc/.../solver/body` | 재활용 |
| 리타게팅 | 손 dex_retargeting | `teleop_dev gen3a` + GR00T | 통합 |
| 제어 | SONIC WBC policy | `GR00T gear_sonic` | 재활용 (checkpoint) |
| 시뮬 A | MuJoCo + ZMQ | `GR00T decoupled_wbc/sim2mujoco` | 재활용 |
| 시뮬 B | Isaac Lab DDS | `unitree_sim_isaaclab` | 재활용 (도커) + 브리지 신규 |
| 배포 | C++ deploy / DDS | `GR00T gear_sonic_deploy` | 2차 |

### 2.3 통합 핵심 — `BaseStreamer` 어댑터 패턴
GR00T-WBC teleop 파이프라인은 입력 장비를 `BaseStreamer` 추상클래스로 추상화한다:
```python
class BaseStreamer(ABC):
    def start_streaming(self): ...
    def get(self) -> StreamerOutput: ...   # ik_data / control_data / teleop_data
    def stop_streaming(self): ...
```
기존 `PicoStreamer`(PICO+XRoboToolkit)와 동일한 출력 포맷을 갖는
**`GalaxyXRStreamer`**를 신규 구현하면, 하드웨어만 교체되고 WBC·IK·sim 파이프라인은
**무변경**으로 재사용된다. 이것이 본 프로젝트의 최소-침습(minimally-invasive) 통합 전략의 핵심이다.

---

## 3. 시뮬레이션 경로 비교 (요구사항 2)

| 기준 | 경로 A: GR00T MuJoCo | 경로 B: unitree_sim_isaaclab |
|---|---|---|
| 엔진 | MuJoCo (경량) | Isaac Lab / Isaac Sim (고품질) |
| 제어 대상 | 전신 (보행+조작) | 주로 상체 조작 task (pick-place 등) |
| 통신 | ZMQ | **DDS (실제 G1과 동일)** |
| 현 상태 | 신규 설치 필요 | **이미 도커 구동 중** (`isaac-lab-base`) |
| 강점 | SONIC 전신 제어 직결, 빠름 | 실로봇 protocol 동일 → sim2real 갭 최소 |
| 약점 | 실로봇 DDS와 별도 | 전신 보행 제어는 별도 정책 필요 |
| 손 지원 | gripper/dex3 | gripper/dex3/inspire/**dg5f(Tesollo)** |

**전략**: 두 경로를 **병행 평가**한다.
- **경로 A**를 *전신 보행 + 조작* 통합 teleop 검증의 1차 경로로 사용 (SONIC 직결).
- **경로 B**를 *상체 조작 + 실로봇 protocol 검증* 및 데이터 수집 경로로 사용 (DDS = 실 G1 동일, 이미 구동 중).
- 공통 입력 어댑터(`GalaxyXRStreamer` → 표준 pose) 위에 경로별 **백엔드 브리지**를 둔다.

---

## 4. 환경 구성 (conda, 요구사항 4)

GR00T 상류는 `uv` 기반 `.venv_sim` / `.venv_teleop` / `.venv_data_collection`을 자동 생성하지만,
로컬 충돌 최소화를 위해 **conda 환경으로 이식**한다.

| conda env | 용도 | 핵심 패키지 | 비고 |
|---|---|---|---|
| `groot_teleop` | 입력 어댑터 + 브리지 + 테스트 (Operator) | numpy<2, scipy, aiohttp, openvr, pyyaml, pynput, dex_retargeting, pink, pin | 본 프로젝트 메인. `teleop_dev` env 기반 |
| `groot_sim` | GR00T MuJoCo 시뮬 (경로 A) | mujoco, GR00T `decoupled_wbc[sim]` | 필요 시 |
| `unitree_sim_env` | Isaac Lab DDS 시뮬 (경로 B) | (도커 내부 기존 env) | **재사용, 신규 생성 안 함** |

- 모든 신규 env는 `envs/*.yaml`로 버전 고정 후 `conda env create -f`.
- 기존 `teleop_operator` env를 베이스로 재사용 가능(이미 dex_retargeting·numpy<2 검증됨).

---

## 5. 개발 원칙 (요구사항 5 · 1·2·3)

1. **선개발 후 테스트**: 하드웨어/실시간 sim이 필요 없는 모든 것을 먼저 개발.
   입력 없이도 도는 **오프라인 replay / 합성 pose 주입 / 단위 테스트**로 자체 검증.
2. **단위별 spike + commit**: 각 작업 단위마다 `docs/spikes/NN-*.md`에 의도·검증·결과를 남기고,
   동작 가능한 단위를 git commit (요구사항 2-1).
3. **2-패스 개발**: 1차 구현 → "전체 개발 계획가" 관점 자체 검토 → 개선 재구현 (요구사항 2-3).
4. **테스트 핸드오프 문서화**: 사용자 개입이 필요한 지점과 그 후 개발 항목을
   `docs/USER_TEST_HANDOFF.md`에 정리 (요구사항 2-4).

### 5.1 Claude가 하드웨어 없이 개발 가능한 범위
- 입력 어댑터(`GalaxyXRStreamer`)와 좌표 변환/프레임 정렬 로직
- 합성 pose 생성기 + 오프라인 replay 하니스 (헤드셋 없이 파이프라인 구동)
- DDS 브리지 (XR pose → G1 관절 IK → DDS publish) 코드 및 단위 테스트
- conda 환경 정의, 설치 스크립트, 프로젝트 스캐폴딩, CI 스모크 테스트
- 문서, 설정, 좌표계 검증용 수치 테스트

### 5.2 사용자 개입이 필수인 범위 (→ `USER_TEST_HANDOFF.md`)
- 실제 Galaxy XR / Quest 3 헤드셋 연결 및 WebXR pose 송출 확인
- Isaac Sim GUI 실시간 시뮬 구동 (도커, GPU 렌더)
- SONIC checkpoint 다운로드(HuggingFace 인증) 및 MuJoCo 실시간 teleop
- 실제 G1 로봇 배포

---

## 6. 리스크 및 대응

| 리스크 | 영향 | 대응 |
|---|---|---|
| GR00T = PICO 전용 streamer | 입력 교체 필요 | `BaseStreamer` 어댑터로 격리 (이미 설계 반영) |
| Galaxy XR WebXR 좌표계 ≠ PICO 좌표계 | 매핑 오류 | `teleop_dev xr_frame_align` 재활용 + 수치 단위테스트 |
| SONIC checkpoint 인증/용량 | 다운로드 지연 | dummy policy로 파이프라인 선검증, 사용자에 인증 위임 |
| 두 sim 경로 분기 비용 | 작업 2배 | 공통 어댑터 + 얇은 백엔드 브리지로 분기 최소화 |
| conda ↔ uv 의존성 차이 | 빌드 실패 | env yaml 버전 고정, 단계별 import 스모크 테스트 |
| Isaac Lab DDS ↔ host 통신 | 네트워크 격리 | 도커 cyclonedds.xml / host net 설정 검증 (사용자 테스트) |

---

## 7. 디렉토리 구조 (목표)

```
groot_teleop_dev/
├── docs/
│   ├── DEVELOPMENT_PLAN.md          # 본 문서
│   ├── WEEKLY_PLAN.md               # 주차별 계획
│   ├── USER_TEST_HANDOFF.md         # 사용자 테스트 인계
│   └── spikes/                      # 단위별 spike 기록
├── envs/                            # conda 환경 정의
│   └── groot_teleop.yaml
├── groot_teleop/                    # 본 프로젝트 패키지
│   ├── input/                       #   입력 어댑터 (XR/Vive → 표준 pose)
│   │   ├── bridge/                  #     teleop_dev xr_common vendoring
│   │   ├── galaxy_xr_streamer.py    #     GR00T BaseStreamer 구현 (신규)
│   │   └── synthetic_streamer.py    #     합성 pose (하드웨어 없이 테스트)
│   ├── backends/                    #   sim 백엔드 브리지
│   │   ├── groot_mujoco/            #     경로 A
│   │   └── unitree_dds/             #     경로 B (XR → IK → DDS)
│   ├── common/                      #   좌표 변환, 설정
│   └── replay/                      #   오프라인 replay 하니스
├── tests/                           # 단위 테스트 (하드웨어 무관)
├── scripts/                         # 실행 스크립트
└── external/                        # 참조 repo (gitignored)
    ├── GR00T-WholeBodyControl/
    ├── teleop_dev/
    └── unitree_sim_isaaclab/
```

---

## 8. 성공 기준 (Definition of Done)

- **M1 (입력)**: 합성 pose로 `GalaxyXRStreamer.get()`이 GR00T `StreamerOutput` 규격을 만족, 단위테스트 통과.
- **M2 (브리지)**: XR pose → G1 상체 관절각 IK 변환이 수치적으로 검증됨 (오프라인).
- **M3 (sim 연결)**: 사용자 테스트로 두 경로 중 1개 이상에서 합성 pose 기반 로봇 팔 동작 확인.
- **M4 (실시간 teleop)**: 실제 헤드셋 → 시뮬 G1 상체 실시간 추종 (사용자 테스트).
- **M5 (전신)**: SONIC WBC로 보행+조작 동시 teleop (사용자 테스트).
- **M6 (실로봇)**: 실제 G1 배포 (최종, 별도 안전 검토).

상세 일정은 [`WEEKLY_PLAN.md`](./WEEKLY_PLAN.md) 참조.
