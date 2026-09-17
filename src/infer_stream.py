"""
웹캠 또는 영상 파일을 이용한 실시간 추론 스크립트.
디지털 트윈 연동 전 실시간 동작을 미리 확인하는 용도.
"""

from ultralytics import YOLO

model = YOLO("yolo11n.pt")

# source=0 -> 맥 기본 웹캠
# source="video.mp4" -> 영상 파일로 대체 가능
results = model.predict(
    source=0,
    device="mps",
    stream=True,   # 프레임 단위로 순차 처리 (메모리 효율적)
    show=True,     # 실시간 창으로 결과 표시
)

for r in results:
    # 프레임별 탐지 결과 처리
    # 여기에 디지털 트윈 서버로 좌표 전송하는 로직을 추가하면 됩니다.
    pass
