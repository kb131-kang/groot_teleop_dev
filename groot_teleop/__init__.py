"""groot_teleop — Galaxy XR / Quest 3 → Unitree G1 whole-body teleoperation.

GR00T-WholeBodyControl (GEAR-SONIC) 제어 백본 위에 teleop_dev 의 WebXR 입력
브리지를 결합한다. 핵심 통합 지점은 GR00T 의 ``BaseStreamer`` 어댑터이다.

서브패키지
    input    — XR/Vive 입력 어댑터 (GR00T BaseStreamer 구현 + 합성 streamer)
    common   — 좌표 변환 / 안전 / 설정
    backends — sim 백엔드 브리지 (unitree DDS / GR00T MuJoCo)
    replay   — 오프라인 replay / 지연 측정 하니스 (하드웨어 무관 테스트)
"""

__version__ = "0.1.0"
