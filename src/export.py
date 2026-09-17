"""
Jetson 배포를 위한 모델 변환 스크립트.

주의: TensorRT(.engine) 변환은 NVIDIA GPU에서만 가능하므로
Mac에서는 ONNX까지만 변환하고, .onnx -> .engine 변환은
Jetson 장치 위에서 직접 실행해야 합니다.
"""

from ultralytics import YOLO

model = YOLO("models/pretrained/yolo11n.pt")

# 1단계: Mac에서 ONNX로 변환
model.export(format="onnx")

print("ONNX 변환 완료.")
print("Jetson으로 옮긴 뒤 아래 명령어로 TensorRT 엔진으로 변환하세요:")
print("  yolo export model=yolo11n.onnx format=engine device=0")
