"""
AI Hub 2.Validation 데이터(학습에 전혀 쓰이지 않은 별도 촬영분)로
모델이 실제로 성장 단계를 잘 맞추는지 검증하는 스크립트.

JSON 안의 growth_stage(정답)와 모델이 예측한 성장 단계를 비교해서
정확도와 클래스별 혼동 현황을 보여줍니다.

사용법:
    python src/utils/evaluate_on_validation.py \
        --images_dir "data/raw/tkdcn/2.Validation/원천데이터" \
        --labels_dir "data/raw/tkdcn/2.Validation/라벨링데이터" \
        --model "runs/train/digital_twin_yolo/weights/best.pt" \
        --max_images 300
"""

import argparse
import json
import unicodedata
from collections import defaultdict
from pathlib import Path

from ultralytics import YOLO

TARGET_CATEGORY_NAME = "주"


def resolve_dir(path_str: str) -> Path:
    p = Path(path_str)
    if p.exists():
        return p
    for form in ("NFC", "NFD"):
        candidate = Path(unicodedata.normalize(form, path_str))
        if candidate.exists():
            return candidate
    return p


def find_image_path(images_dir: Path, fname: str, image_id, fext: str):
    direct = images_dir / fname
    if direct.exists():
        return direct
    matches = list(images_dir.rglob(fname))
    if matches:
        return matches[0]
    if image_id is not None:
        matches = list(images_dir.rglob(f"*_{image_id}.{fext}"))
        if matches:
            return matches[0]
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--images_dir", required=True)
    parser.add_argument("--labels_dir", required=True)
    parser.add_argument("--model", default="runs/train/digital_twin_yolo/weights/best.pt")
    parser.add_argument("--max_images", type=int, default=300, help="검증에 사용할 최대 이미지 수")
    parser.add_argument("--device", default="mps")
    parser.add_argument("--conf", type=float, default=0.25, help="예측 신뢰도 임계값")
    args = parser.parse_args()

    images_dir = resolve_dir(args.images_dir)
    labels_dir = resolve_dir(args.labels_dir)

    json_files = list(labels_dir.rglob("*.json"))
    print(f"Validation JSON 파일 {len(json_files)}개 발견")

    if not json_files:
        print("JSON 파일을 찾지 못했습니다. --labels_dir 경로를 확인하세요.")
        return

    # 이미지-정답 쌍 수집
    pairs = []  # (img_path, true_stage)
    skipped_no_image = 0

    for jf in json_files:
        try:
            with open(jf, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            continue

        img_info = data.get("images", {})
        fname = img_info.get("fname")
        image_id = img_info.get("image_id")
        fext = img_info.get("fext", "jpg")
        growth_stage = img_info.get("growth_stage")

        if not growth_stage:
            continue

        img_path = find_image_path(images_dir, fname, image_id, fext)
        if img_path is None:
            skipped_no_image += 1
            continue

        pairs.append((img_path, growth_stage))

        if len(pairs) >= args.max_images:
            break

    print(f"검증에 사용할 이미지: {len(pairs)}개 (이미지 못 찾아서 제외: {skipped_no_image}개)")

    if not pairs:
        print("검증할 이미지가 없습니다. --images_dir 경로나 다운로드 상태를 확인하세요.")
        return

    # 모델 로드
    model = YOLO(args.model)
    print(f"\n모델 로드 완료: {args.model}")
    print(f"클래스 목록: {model.names}\n")

    # 예측 및 비교
    correct = 0
    total = 0
    confusion = defaultdict(lambda: defaultdict(int))  # true -> predicted -> count
    no_detection = 0

    for img_path, true_stage in pairs:
        results = model.predict(source=str(img_path), device=args.device, conf=args.conf, verbose=False)
        r = results[0]

        if len(r.boxes) == 0:
            no_detection += 1
            confusion[true_stage]["탐지실패"] += 1
            continue

        # 가장 신뢰도 높은 박스의 클래스를 예측값으로 사용
        best_box = max(r.boxes, key=lambda b: float(b.conf[0]))
        pred_stage = model.names[int(best_box.cls[0])]

        confusion[true_stage][pred_stage] += 1
        total += 1
        if pred_stage == true_stage:
            correct += 1

    # 결과 출력
    print("=== 클래스별 혼동 현황 (정답 -> 예측:개수) ===")
    for true_stage in sorted(confusion.keys()):
        preds = confusion[true_stage]
        pred_summary = ", ".join(f"{p}:{c}" for p, c in sorted(preds.items(), key=lambda x: -x[1]))
        print(f"  {true_stage} -> {pred_summary}")

    print(f"\n탐지 실패(박스를 아예 못 찾음): {no_detection}개")
    if total > 0:
        accuracy = correct / total * 100
        print(f"\n=== 전체 정확도: {correct}/{total} ({accuracy:.1f}%) ===")
    else:
        print("\n비교 가능한 예측이 없습니다.")


if __name__ == "__main__":
    main()