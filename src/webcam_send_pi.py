"""
라즈베리파이 CSI 카메라로 여러 화분의 상추를 촬영하고, 화분마다 따로 성장 단계를 추론해서
'수확 가능 여부(harvest)'와 함께 Unity PC로 UDP 전송하는 스크립트.
라즈베리파이에서 실행하세요.

사용법 (venv 활성화 후):
    python webcam_send_pi.py

화분 구분 방식:
    카메라와 화분을 고정해두고, 화면을 화분별 구역(POT_ZONES)으로 나눠서
    구역 번호 = 화분 ID로 사용합니다.

Unity로 가는 메시지(JSON) 예시 (화분 하나당 1건):
    {"pot_id": 1, "stage": "수확기", "confidence": 0.93, "harvest": true, "time": 1793260000.1}
"""

import json
import socket
import time
from collections import deque

print("라이브러리 불러오는 중... (처음에는 1분 정도 걸릴 수 있어요)", flush=True)

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

# ===== 설정 (여기만 바꾸면 됨) =====
SEND_TO_UNITY = True             # False로 두면 전송 없이 콘솔 출력만
UNITY_IP = "192.168.0.100"       # Unity PC의 IP 주소
UNITY_PORT = 60000               # UDP 포트
MODEL_PATH = "best_ncnn_model"   # 같은 폴더에 옮겨둔 NCNN 모델 폴더
INTERVAL_SEC = 5                 # 한 바퀴 처리 후 쉬는 시간(초)
CONF_THRESHOLD = 0.25            # 이 신뢰도 미만의 탐지는 무시
CAPTURE_SIZE = (1280, 720)       # 촬영 해상도. 화분이 4개 이상이면 (2304, 1296) 권장
SAVE_LAST_IMAGE = True           # 마지막 결과를 last_result.jpg 한 장에 덮어써서 저장
DEBUG_STEPS = True               # 진행 단계를 출력 (멈추는 위치 확인용, 안정되면 False)

# 화분 구역: {화분 ID: (왼쪽, 위, 오른쪽, 아래)}, 값은 화면 크기에 대한 0~1 비율
# 화분 1개:                {1: (0.0, 0.0, 1.0, 1.0)}
# 가로로 3개:              {1: (0.0, 0.0, 0.33, 1.0), 2: (0.33, 0.0, 0.66, 1.0), 3: (0.66, 0.0, 1.0, 1.0)}
# 2x2 배치:                {1: (0.0, 0.0, 0.5, 0.5), 2: (0.5, 0.0, 1.0, 0.5),
#                           3: (0.0, 0.5, 0.5, 1.0), 4: (0.5, 0.5, 1.0, 1.0)}
POT_ZONES = {1: (0.0, 0.0, 1.0, 1.0)}

# 추론 방식
#   "crop": 구역마다 잘라서 각각 추론 (정확한 편, 화분 수만큼 느림) - 추천
#   "full": 화면 전체를 한 번에 추론하고 박스 위치로 화분을 배정 (빠름)
INFER_MODE = "crop"

# 수확 판정 기준: 아래 조건이 연속으로 충족돼야 harvest = True
HARVEST_STAGE = "수확기"          # 이 단계로 판정되고
HARVEST_CONF = 0.6               # 신뢰도가 이 값 이상인 경우가
HARVEST_CONFIRM_COUNT = 3        # 이 횟수만큼 연속일 때 (한 바퀴가 약 5초면 약 15초)
# ====================================

# 결과 이미지에 쓸 영문 단계 이름 (cv2는 한글을 그리지 못함)
STAGE_EN = {"정식기": "planting", "생육기": "growing", "수확기": "harvest", "없음": "none"}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
history = {pid: deque(maxlen=HARVEST_CONFIRM_COUNT) for pid in POT_ZONES}


def step(msg):
    if DEBUG_STEPS:
        print(msg, flush=True)


def send_result(pot_id, stage, conf, harvest):
    if not SEND_TO_UNITY:
        return
    msg = json.dumps(
        {
            "pot_id": pot_id,
            "stage": stage,
            "confidence": round(conf, 3),
            "harvest": harvest,
            "time": time.time(),
        },
        ensure_ascii=False,
    )
    sock.sendto(msg.encode("utf-8"), (UNITY_IP, UNITY_PORT))


def update_harvest(pot_id, stage, conf):
    """이번 결과를 기록하고, 연속 확인 조건을 만족하면 True를 반환."""
    is_ripe = stage == HARVEST_STAGE and conf >= HARVEST_CONF
    history[pot_id].append(is_ripe)
    return len(history[pot_id]) == HARVEST_CONFIRM_COUNT and all(history[pot_id])


def zone_pixels(zone, w, h):
    x0, y0, x1, y1 = zone
    return int(x0 * w), int(y0 * h), int(x1 * w), int(y1 * h)


def detect_full(model, frame):
    """화면 전체를 한 번에 추론하고, 박스 중심이 속한 구역으로 화분을 배정한다.
    반환: {화분 ID: (단계, 신뢰도, (x1, y1, x2, y2))}"""
    r = model.predict(
        source=frame, conf=CONF_THRESHOLD, agnostic_nms=True, verbose=False
    )[0]
    found = {}
    for box in r.boxes:
        xc, yc = float(box.xywhn[0][0]), float(box.xywhn[0][1])
        conf = float(box.conf[0])
        for pot_id, (x0, y0, x1, y1) in POT_ZONES.items():
            if x0 <= xc <= x1 and y0 <= yc <= y1:
                if pot_id not in found or conf > found[pot_id][1]:
                    xyxy = tuple(int(v) for v in box.xyxy[0].tolist())
                    found[pot_id] = (model.names[int(box.cls[0])], conf, xyxy)
                break
    return found


def detect_crop(model, frame):
    """구역마다 잘라서 각각 추론한다. 학습 데이터(상추 한 포기가 화면에 가득)와
    비슷한 모양이 되어 판정이 안정적이다.
    반환: {화분 ID: (단계, 신뢰도, (x1, y1, x2, y2))}  (좌표는 원본 화면 기준)"""
    h, w = frame.shape[:2]
    found = {}
    for pot_id, zone in POT_ZONES.items():
        px0, py0, px1, py1 = zone_pixels(zone, w, h)
        crop = frame[py0:py1, px0:px1].copy()
        if crop.size == 0:
            continue
        r = model.predict(
            source=crop, conf=CONF_THRESHOLD, agnostic_nms=True, verbose=False
        )[0]
        if len(r.boxes) == 0:
            continue
        best = max(r.boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = (int(v) for v in best.xyxy[0].tolist())
        found[pot_id] = (
            model.names[int(best.cls[0])],
            float(best.conf[0]),
            (x1 + px0, y1 + py0, x2 + px0, y2 + py0),
        )
    return found


def draw_overlay(frame, found, flags):
    """구역(초록, 수확 가능이면 빨강)과 탐지 박스(노랑), 결과 글자를 그려서 반환."""
    img = frame.copy()
    h, w = img.shape[:2]
    for pot_id, zone in POT_ZONES.items():
        px0, py0, px1, py1 = zone_pixels(zone, w, h)
        ready = flags.get(pot_id, False)
        color = (0, 0, 255) if ready else (0, 200, 0)  # BGR
        cv2.rectangle(img, (px0, py0), (px1, py1), color, 2)
        if pot_id in found:
            stage, conf, (x1, y1, x2, y2) = found[pot_id]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
            label = f"P{pot_id} {STAGE_EN.get(stage, stage)} {conf:.2f}"
        else:
            label = f"P{pot_id} none"
        if ready:
            label += " HARVEST"
        cv2.putText(
            img, label, (px0 + 6, py0 + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
        )
    return img


def main():
    step("모델 불러오는 중...")
    model = YOLO(MODEL_PATH)
    step("모델 로드 완료")

    step("카메라 시작 중...")
    picam2 = Picamera2()
    # picamera2의 RGB888은 OpenCV와 같은 BGR 순서로 나와서 YOLO에 그대로 넣을 수 있음
    config = picam2.create_still_configuration(
        main={"size": CAPTURE_SIZE, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # 카메라 워밍업
    step("카메라 준비 완료. 촬영을 시작합니다.")

    save_image = SAVE_LAST_IMAGE

    try:
        while True:
            # 오토포커스 카메라(Camera Module 3)라서 촬영 전에 초점을 한 번 맞춤
            step("초점 맞추는 중...")
            try:
                picam2.autofocus_cycle()
            except Exception:
                pass  # 오토포커스가 없는 카메라면 건너뜀

            step("촬영 중...")
            frame = picam2.capture_array()

            step("추론 중...")
            t0 = time.time()
            if INFER_MODE == "full":
                found = detect_full(model, frame)
            else:
                found = detect_crop(model, frame)
            elapsed = time.time() - t0

            flags = {}
            for pot_id in POT_ZONES:
                if pot_id in found:
                    stage, conf, _ = found[pot_id]
                else:
                    stage, conf = "없음", 0.0

                harvest = update_harvest(pot_id, stage, conf)
                flags[pot_id] = harvest
                print(
                    f"[화분 {pot_id}] {stage} (신뢰도 {conf:.2f}) "
                    f"수확 판정: {'예' if harvest else '아니오'}"
                )
                send_result(pot_id, stage, conf, harvest)
            print(f"-- 한 바퀴 추론 {elapsed:.2f}초 --")

            if save_image:
                try:
                    cv2.imwrite("last_result.jpg", draw_overlay(frame, found, flags))
                except Exception as e:
                    print(f"결과 이미지 저장 실패, 이후 저장은 건너뜁니다: {e}")
                    save_image = False

            time.sleep(INTERVAL_SEC)
    except KeyboardInterrupt:
        print("종료합니다.")
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()