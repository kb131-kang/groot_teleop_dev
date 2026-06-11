# 경로 A — GR00T-WBC MuJoCo teleop 통합 가이드

GR00T-WBC 의 MuJoCo/ZMQ teleop 파이프라인에 `GalaxyXRStreamer` 를 PICO drop-in
으로 결선한다. `external/` 은 gitignored 이므로 본 패치는 **문서로 제공**하고,
사용자가 GR00T 설치본에 적용한다.

## 패치 위치
`decoupled_wbc/control/teleop/teleop_streamer.py` 의 `TeleopStreamer.__init__`
내 `body_control_device` 분기 (pico/vive/iphone... 옆).

## 패치 (1 분기 추가)
```python
# decoupled_wbc/control/teleop/teleop_streamer.py
elif body_control_device == "galaxy_xr":
    from groot_teleop.input.galaxy_xr_streamer import GalaxyXRStreamer
    self.body_streamer = GalaxyXRStreamer()   # BridgePoseStore ws 자동 기동
    self.body_streamer.start_streaming()
```

`GalaxyXRStreamer.get()` 는 `PicoStreamer.get()` 과 **동일한 StreamerOutput**
(`ik_data{left_wrist,right_wrist,left/right_fingers}`, `control_data{base_height_command,
navigate_cmd,toggle_policy_action}`, `teleop_data`, `data_collection_data`)을 내므로
하위 `WristsPreProcessor` / `FingersPreProcessor` / IK / WBC 는 무변경 동작한다.

## 설치 전제
1. GR00T MuJoCo 환경(`install_scripts/install_mujoco_sim.sh`) 또는 conda `groot_sim`.
2. 같은 환경에 본 패키지 설치: `pip install -e .` (groot_teleop import 가능해야 함).
3. SONIC checkpoint 다운로드 (HuggingFace 인증) — `download_from_hf.py`.

## 좌표계 정합
- `GalaxyXRStreamer` 는 PicoStreamer 와 동일한 z-up + headset-relative + yaw 보상
  (`common/frames.py`) 을 적용 → PICO 와 같은 좌표 규약. 추가 정합 불필요(이론상).
- 단, WebXR 손목 frame 과 PICO controller frame 의 **회전 offset** 차이가 있을 수
  있어 USER_TEST 에서 손 방향 확인 필요 (필요 시 `frames` 에 회전 offset 추가).

## 제한 / 후속
- 이동(보행) 명령: hand-tracking 모드에는 joystick 이 없어 `navigate_cmd=0`.
  보행은 (a) Vive 발목 트래커(Phase 4) 또는 (b) 별도 입력으로 결선 예정(Phase 3.5).
- base height 조절: 현재 고정(0.74). 제스처/버튼 매핑은 Phase 3.
