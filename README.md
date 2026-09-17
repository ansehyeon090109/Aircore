# Digital Twin YOLO (Mac)

디지털 트윈 연동용 YOLO11 객체 인식 프로젝트 (M3 Mac 개발 → Jetson 배포)

## 1. 초기 설정

```bash
# 가상환경 생성 및 활성화
python3 -m venv venv
source venv/bin/activate

# 패키지 설치
pip install -r requirements.txt
```

VS Code에서:
1. `Cmd+Shift+P` → `Python: Select Interpreter` → `./venv/bin/python` 선택
2. 확장 설치: Python, Pylance, Jupyter, Remote-SSH

## 2. 동작 확인 (첫 실행)

```bash
python src/infer.py
```

- 첫 실행 시 `yolo11n.pt` 가중치가 자동 다운로드됩니다.
- 결과 이미지는 `runs/detect` 폴더에 저장됩니다 (매번 같은 폴더에 덮어씀).
- M3의 MPS(GPU 가속)를 기본으로 사용합니다.

## 3. 웹캠 실시간 테스트

```bash
python src/infer_stream.py
```

## 4. 자체 데이터로 파인튜닝 (상추 등)

1. 이미지에 라벨(바운딩 박스)이 없다면 Roboflow(roboflow.com) 등으로 먼저 라벨링
2. `data/datasets/data.yaml`에 클래스와 이미지 경로 설정 (현재 `lettuce` 예시로 되어 있음)
3. 학습 실행:
```bash
python src/train.py
```

## 5. Jetson 배포용 변환

Mac에서 ONNX까지 변환:
```bash
python src/export.py
```

이후 Jetson 장치에서 TensorRT 엔진으로 변환 (Jetson 위에서 실행):
```bash
yolo export model=yolo11n.onnx format=engine device=0
```

## 폴더 구조

```
digital-twin-yolo/
├── data/
│   ├── raw/            # 원본 이미지/영상
│   └── datasets/        # 학습용 정리 데이터 + data.yaml
├── models/
│   ├── pretrained/       # 사전학습 가중치(.pt)
│   └── exported/         # 변환된 모델(.onnx, .engine)
├── src/
│   ├── infer.py          # 이미지 추론 테스트
│   ├── infer_stream.py   # 웹캠 실시간 추론
│   ├── train.py          # 파인튜닝
│   └── export.py         # 모델 변환
└── notebooks/            # 실험/시각화용 Jupyter 노트북
```
