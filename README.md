# groot_teleop_dev

Galaxy XR / Meta Quest 3 → **Unitree G1** 휴머노이드 전신 원격조종.
제어 백본은 NVlabs [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl)
(GEAR-SONIC whole-body control), 입력 계층은 자체 개발 `teleop_dev` 의 WebXR 브리지를 재활용한다.

## 구조

```
groot_teleop/
├── input/        XR/Vive 입력 어댑터 (GR00T BaseStreamer 구현)
│   ├── bridge/                 WebXR ws 브리지 (Galaxy XR/Quest 3, teleop_dev vendoring)
│   ├── galaxy_xr_streamer.py   GalaxyXRStreamer — PICO drop-in 대체
│   └── synthetic_streamer.py   헤드셋 없이 pose 합성 (테스트용)
├── common/       좌표 변환(frames.py) / 안전 / 설정
├── backends/     sim 백엔드 브리지 (unitree DDS · GR00T MuJoCo)
└── replay/       오프라인 replay / 지연 측정
```

## 문서
- [개발 계획서](docs/DEVELOPMENT_PLAN.md) · [주차별 계획](docs/WEEKLY_PLAN.md)
- [사용자 테스트 인계](docs/USER_TEST_HANDOFF.md) · [spike 기록](docs/spikes/)

## 빠른 시작 (개발/테스트, 헤드셋 무관)

```bash
# 1) conda 환경
conda env create -f envs/groot_teleop.yaml
conda activate groot_teleop
pip install -e .

# 2) 단위테스트 (하드웨어 불필요)
pytest tests/ -v

# 3) WebXR 브리지 selftest (헤드셋 없이 server 부팅 확인)
python -m groot_teleop.input.bridge.bridge_pose_store --selftest
```

> 참조 repo 는 `external/` 에 clone (gitignored). `DEVELOPMENT_PLAN.md` §7 참조.
