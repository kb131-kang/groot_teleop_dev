"""BaseStreamer / StreamerOutput 계약 (GR00T-WholeBodyControl 호환).

GR00T 의 ``decoupled_wbc/control/teleop/streamers/base_streamer.py`` 와
**바이트 단위로 호환되는** 계약을 vendoring 한다. 목적:

  1. GR00T env 가 설치된 곳에서는 실제 GR00T BaseStreamer 를 그대로 사용.
  2. GR00T 미설치 환경(=하드웨어 무관 단위테스트)에서는 이 vendored 사본을 사용.

따라서 ``GalaxyXRStreamer`` 등은 ``from groot_teleop.input.base_streamer import
BaseStreamer, StreamerOutput`` 로 import 하면, 이 shim 이 GR00T 가 있으면 그쪽을,
없으면 vendored 사본을 자동 선택한다 — drop-in 호환.

⚠️ GR00T 가 StreamerOutput 필드를 바꾸면 이 사본도 동기화해야 한다.
   (현재 기준: ik_data / control_data / teleop_data / data_collection_data /
    timestamp / source)
"""

from __future__ import annotations

# ── GR00T 가 설치돼 있으면 그쪽 계약을 우선 사용 (완전 호환 보장) ────────────
try:  # pragma: no cover - 환경 의존
    from decoupled_wbc.control.teleop.streamers.base_streamer import (  # type: ignore
        BaseStreamer,
        StreamerOutput,
    )

    _USING_GROOT = True
except Exception:  # GR00T 미설치 — vendored 사본 사용
    _USING_GROOT = False

    from abc import ABC, abstractmethod
    from dataclasses import dataclass, field
    import time
    from typing import Any, Dict

    @dataclass
    class StreamerOutput:
        """입력 장비의 한 frame 을 GR00T teleop 파이프라인 4종 데이터로 분류.

        ik_data             — IK 처리가 필요한 데이터 (손목 4×4 pose, 손가락)
        control_data        — robot control loop 로 직접 전달 (base 높이, 이동 명령)
        teleop_data         — teleop policy 내부용 (activation toggle 등)
        data_collection_data— 데이터 수집 제어 (record/abort toggle)
        """

        ik_data: Dict[str, Any] = field(default_factory=dict)
        control_data: Dict[str, Any] = field(default_factory=dict)
        teleop_data: Dict[str, Any] = field(default_factory=dict)
        data_collection_data: Dict[str, Any] = field(default_factory=dict)
        timestamp: float = field(default_factory=time.time)
        source: str = ""

    class BaseStreamer(ABC):
        def __init__(self, *args, **kwargs):
            pass

        def reset_status(self):
            pass

        @abstractmethod
        def start_streaming(self):
            ...

        @abstractmethod
        def get(self) -> "StreamerOutput":
            """structured StreamerOutput 반환."""
            ...

        @abstractmethod
        def stop_streaming(self):
            ...


def using_groot() -> bool:
    """현재 실제 GR00T BaseStreamer 를 쓰는지(True) vendored 사본인지(False)."""
    return _USING_GROOT


__all__ = ["BaseStreamer", "StreamerOutput", "using_groot"]
