# Spike 03 — 개선 패스 (전체 계획가 검토 → 재개발)

**요구사항 2-3** · 상태: ✅ 완료 (Claude 단독)

## 검토 관점
"전체 개발 계획가" 관점에서 Phase 1–2 구현을 검토하고, **실사용 가능한 teleop**
이 되려면 빠진 핵심을 식별해 재개발.

## 식별된 개선점 & 조치
| # | 문제 | 조치 | 검증 |
|---|---|---|---|
| 1 | 안전장치 없음 — 비현실적 target 으로 IK 발산/자기충돌 가능 | `IKTargetMapper` 에 **G1 workspace clamp**(±0.6m envelope) 추가 | `test_workspace_clamp_bounds_target` |
| 2 | absolute 매핑만 — 사용자가 손 위치 바꾸면 로봇 점프 | **relative-motion + recalibrate** 모드 추가 (`frames.relative_motion`) | `test_relative_mode_origin_invariant` |
| 3 | pose 끊김 시 stale target 송신 위험 | run loop 에 **StoreWatchdog hold** 결선 (실 헤드셋 모드) | dry-run 스모크 |
| 4 | estop/정지 경로 없음 | `bridge.hold()` (직전 팔자세 유지 publish) | `test_hold_keeps_last_arm` |
| 5 | GR00T(경로 A) 결선 미문서화 | `backends/groot_mujoco/INTEGRATION.md` (1-분기 패치) | — |

## 검토 중 드러난 설계 사실 (테스트가 잡음)
- **relative 모드의 origin 불변성은 robot_origin 이 동일할 때만 성립.**
  테스트가 처음엔 실패 → robot_origin 을 absolute 첫 스텝 IK 로 부트스트랩해
  mapper origin 에 의존했기 때문. 수정: robot_origin 은 **로봇 상태 피드백**
  에서 와야 한다(설계 정설).
  - ⚠️ **알려진 근사**: 현재 `recalibrate()` 는 robot_origin 을 *직전 명령 q 의
    FK*(open-loop)로 잡는다. 폐루프로 가려면 **rt/lowstate** 를 구독해 실제
    palm pose 를 써야 한다 → 후속(아래).

## 회귀 검증
```
pytest tests/  → 33 passed (개선으로 +3)
python scripts/run_unitree_dds_teleop.py --dry-run --reachable-demo --relative  → conv=True
```

## 후속 (다음 개발 단계)
- `rt/lowstate` 구독 → 폐루프 robot_origin + 현재 q warm-start (USER_TEST 후).
- 손(dex3/dg5f) 리타게팅 DDS 연결 (Phase 2.6).
- WebXR↔PICO 손목 회전 offset 정합 (USER_TEST 에서 측정 후 `frames` 반영).
