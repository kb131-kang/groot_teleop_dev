# Spike 01 — XR 입력 어댑터 (Galaxy XR / Quest 3 → GR00T)

**Phase 1 / Unit 1.1–1.5** · 상태: ✅ 완료 (Claude 단독, 헤드셋 무관)

## 목표
실제 헤드셋 없이, Galaxy XR/Quest 3 WebXR pose 를 GR00T `StreamerOutput` 으로
변환하는 어댑터를 구현하고 수치적으로 검증한다.

## 구현
| 파일 | 역할 |
|---|---|
| `input/bridge/` | teleop_dev `xr_common` vendoring (BridgePoseStore + webxr_to_pose.html + watchdog) |
| `input/base_streamer.py` | GR00T BaseStreamer/StreamerOutput 계약 shim (GR00T 있으면 그쪽, 없으면 vendored) |
| `common/frames.py` | WebXR(y-up) → z-up, headset-relative + yaw 보상 좌표 변환 (PicoStreamer 수치 동일) |
| `input/synthetic_streamer.py` | `SyntheticPoseSource` — 헤드셋 없이 pose 합성 (BridgePoseStore 호환 인터페이스) |
| `input/galaxy_xr_streamer.py` | `GalaxyXRStreamer(BaseStreamer)` — PICO drop-in 대체 |

## 핵심 설계
- **좌표 변환 분리**: `frames.py` 를 순수 함수로 빼서 헤드셋 없이 단위테스트.
  PicoStreamer `_process_xr_pose` 와 동일 알고리즘을 4×4 입력용으로 이식.
- **pose_source 주입**: GalaxyXRStreamer 가 `pose_source` 를 받음 → 실제(BridgePoseStore)
  /합성(SyntheticPoseSource) 교체로 동일 코드 경로 테스트.
- **그리퍼**: WebXR pinch 거리로 구동 (PICO trigger/grip 대체).

## 검증 (헤드셋 없이)
```bash
conda run -n teleop_operator python -m pytest tests/ -v        # 15 passed
conda run -n teleop_operator python -m groot_teleop.input.bridge.bridge_pose_store --selftest  # PASS
```
- 좌표 변환 8종 (축 매핑/SE3 보존/yaw 보상/relative-motion) ✅
- StreamerOutput 계약 (키·형상) ✅
- pinch→그리퍼, 결정성, all-zero 방어 ✅

## 검증된 수치 사실
- WebXR +Y→world +Z, -Z→+X, +X→-Y (z-up 매핑 정확).
- 머리 yaw 90° 회전해도 '정면' 손목 target 은 world +X 로 정규화 (yaw 보상 동작).

## 다음 (🧑 사용자 테스트 → `USER_TEST_HANDOFF.md` 1.6)
- 실제 Galaxy XR/Quest 3 USB-C 연결 → `adb reverse tcp:8013` → Chrome 에서
  `http://localhost:8013/` → Enter VR → pose 송출 확인.
- 그 후: GR00T `TeleopStreamer` 에 `galaxy_xr` 분기 1줄 추가 (Spike 02 이후).
