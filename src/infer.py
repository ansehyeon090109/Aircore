"""
기본 동작 확인용 추론 스크립트.
YOLO11n 사전학습 모델(COCO)로 샘플 이미지에 대해 객체 탐지를 실행합니다.
처음 실행 시 yolo11n.pt 가중치가 자동으로 다운로드됩니다.
"""

from ultralytics import YOLO

# 1. 모델 로드 (COCO 사전학습 가중치)
model = YOLO("yolo11n.pt")

# 2. 추론 실행
# device="mps"  -> M3 GPU 가속 사용 (Apple Silicon)
# device="cpu"  -> MPS 문제 있을 경우 대체용
results = model.predict(
    source="https://ultralytics.com/images/bus.jpg",
    device="mps",
    save=True,
    project="runs",       # 저장 상위 폴더
    name="detect",         # 폴더명 고정
    exist_ok=True,          # 매번 새 폴더 만들지 않고 덮어쓰기
)

print("추론 완료!www runs/detect 폴더에서 결과 이미지를 확인하세요.")

for r in results:
    print(f"탐지된 객체 수: {len(r.boxes)}")
    for box in r.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        cls_name = model.names[cls_id]
        print(f"  - {cls_name} (신뢰도: {conf:.2f})")
