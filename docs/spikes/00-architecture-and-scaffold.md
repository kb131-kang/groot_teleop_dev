# Spike 00 — 아키텍처 확정 & 프로젝트 스캐폴딩

**Phase 0 / Unit 0.1–0.3** · 상태: ✅ 완료 (Claude 단독)

## 목표
3개 참조 repo를 분석해 통합 아키텍처를 확정하고, conda 기반 프로젝트 골격을 만든다.

## 조사 결과 (핵심)
- **GR00T-WBC**: teleop 입력이 `BaseStreamer.get() → StreamerOutput` 으로 추상화됨.
  `PicoStreamer` 가 PICO+XRoboToolkit 구현체. → 입력 장비 교체 = streamer 교체만으로 가능.
- **teleop_dev** (자체 개발): `sender/xr_common/BridgePoseStore` 가 Galaxy XR/Quest 3 의
  WebXR pose 를 aiohttp ws 로 받아 4×4 SE(3) 로 제공. 헤드셋 없이 selftest 가능.
- **unitree_sim_isaaclab** (자체 fork): Isaac Lab + **DDS**(실 G1 동일), 이미 도커 구동 중.

## 결정
1. **최소-침습 통합**: `GalaxyXRStreamer(BaseStreamer)` 를 신규 구현해 PICO 를 drop-in 대체.
   IK·WBC·sim 파이프라인은 무변경 재사용.
2. **conda** 로 환경 이식 (GR00T 상류는 uv). `envs/groot_teleop.yaml` = teleop_operator 기반.
3. **sim 2경로 병행**: (A) GR00T MuJoCo, (B) unitree DDS. 공통 입력 어댑터 + 얇은 백엔드 브리지.

## 산출물
- `docs/DEVELOPMENT_PLAN.md`, `docs/WEEKLY_PLAN.md`
- `groot_teleop/` 패키지 골격, `envs/groot_teleop.yaml`, `pyproject.toml`, `.gitignore`
- `external/` 에 3개 참조 repo clone (gitignored)

## 검증
- 패키지 import 구조 정상. (테스트는 spike 01 에서)

## 다음
- Spike 01: 입력 어댑터 구현 + 단위테스트.
