"""
자체 데이터로 파인튜닝할 때 사용하는 학습 스크립트.
data/datasets/data.yaml 파일을 먼저 준비해야 합니다.
"""

from ultralytics import YOLO

model = YOLO("yolo11n.pt")  # nano 모델부터 시작 (Jetson 배포 고려)

model.train(
    data="data/datasets/data.yaml",
    epochs=50,        # 데이터가 훨씬 많아져서 100 -> 50으로 줄임
    imgsz=640,
    batch=32,          # 데이터가 많으니 배치 크기 키워서 학습 속도 개선
    device="mps",      # M3 GPU 가속. 문제 있으면 "cpu"로 변경
    project="runs/train",
    name="digital_twin_yolo",
    exist_ok=True,      # 재실행해도 새 폴더(_2, _3...) 안 만들고 덮어씀
    patience=15,        # 15 epoch 동안 개선 없으면 조기 종료 (시간 절약)
)