"""
AI Hub 상추 데이터셋의 JSON 라벨을 YOLO 형식(txt)으로 변환하는 스크립트.

- category name이 "주"(식물체 전체)인 어노테이션만 사용
- 클래스는 고정된 "lettuce" 하나가 아니라, JSON의 growth_stage
  (예: 파종기/생육기/수확기 등)를 그대로 사용 -> 성장 단계 분류까지 겸함
- COCO 방식의 bbox [x_min, y_min, width, height] (픽셀 단위)를
  YOLO 방식 [x_center, y_center, width, height] (0~1 정규화)로 변환
- 이미지와 라벨을 train/val로 자동 분리하여
  data/datasets/images/{train,val}, data/datasets/labels/{train,val} 에 정리

사용법:
    python src/utils/convert_json_to_yolo.py --images_dir "data/raw/tkdcn/2.Validation/원천데이터" --labels_dir "data/raw/tkdcn/2.Validation/라벨링데이터" --output_dir "data/datasets" \--val_ratio 0.2

이미지 폴더와 라벨(JSON) 폴더가 같은 경우, 두 인자에 같은 경로를 넣어도 됩니다.
"""

import argparse
import json
import random
import shutil
import unicodedata
from pathlib import Path

TARGET_CATEGORY_NAME = "주"   # 식물체 전체를 나타내는 카테고리


def convert_bbox(bbox, img_w, img_h):
    """COCO 스타일 [x_min, y_min, w, h] (픽셀) -> YOLO 스타일 [x_c, y_c, w, h] (0~1 정규화)"""
    x_min, y_min, w, h = bbox
    x_center = (x_min + w / 2) / img_w
    y_center = (y_min + h / 2) / img_h
    norm_w = w / img_w
    norm_h = h / img_h
    return x_center, y_center, norm_w, norm_h


def process_json_file(json_path: Path):
    """
    JSON 파일 하나를 읽어서 (이미지 파일명, image_id, fext, growth_stage, bbox 리스트)를 반환.
    못 찾으면 None.
    """
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"  [경고] JSON 읽기 실패: {json_path.name} ({e})")
        return None

    img_info = data.get("images")
    if not img_info:
        print(f"  [경고] images 정보 없음: {json_path.name}")
        return None

    fname = img_info.get("fname")
    image_id = img_info.get("image_id")
    fext = img_info.get("fext", "jpg")
    img_w = float(img_info.get("width"))
    img_h = float(img_info.get("height"))
    growth_stage = img_info.get("growth_stage")

    # category id -> name 매핑
    categories = {c["id"]: c["name"] for c in data.get("categories", [])}

    boxes = []
    for ann in data.get("annotations", []):
        cat_name = categories.get(ann.get("category_id"))
        if cat_name != TARGET_CATEGORY_NAME:
            continue

        x_c, y_c, w, h = convert_bbox(ann["bbox"], img_w, img_h)

        # 좌표가 0~1 범위를 벗어나면 보정 (라벨 오류 방지)
        x_c = min(max(x_c, 0.0), 1.0)
        y_c = min(max(y_c, 0.0), 1.0)
        w = min(max(w, 0.0), 1.0)
        h = min(max(h, 0.0), 1.0)

        boxes.append((x_c, y_c, w, h))

    return fname, image_id, fext, growth_stage, boxes


def find_image_path(images_dir: Path, fname: str, image_id, fext: str):
    """
    images_dir 아래(하위 폴더 포함)에서 이미지를 찾는다.

    JSON의 fname(촬영 타임스탬프 기반 이름)과 실제 배포된 이미지 파일명이
    다른 경우가 있어서(AI Hub 상추 데이터셋 특성), 아래 순서로 시도한다:
      1. fname 그대로 일치하는 파일
      2. "*_{image_id}.{fext}" 패턴으로 끝나는 파일 (image_id 기준 매칭)
    """
    # 1. fname 그대로 시도
    direct = images_dir / fname
    if direct.exists():
        return direct
    matches = list(images_dir.rglob(fname))
    if matches:
        return matches[0]

    # 2. image_id 기준으로 재시도
    if image_id is not None:
        pattern = f"*_{image_id}.{fext}"
        matches = list(images_dir.rglob(pattern))
        if matches:
            return matches[0]

    return None


def resolve_dir(path_str: str) -> Path:
    """
    입력 경로가 그대로 존재하지 않으면, 유니코드 정규화(NFC/NFD)를 바꿔가며 재시도한다.
    macOS는 한글 파일/폴더명을 NFD(분해형)로 저장하는 경우가 많아서,
    터미널에 붙여넣은 NFC(완성형) 경로와 안 맞아 존재하지 않는 것처럼 보일 수 있다.
    """
    p = Path(path_str)
    if p.exists():
        return p

    for form in ("NFC", "NFD"):
        normalized = unicodedata.normalize(form, path_str)
        candidate = Path(normalized)
        if candidate.exists():
            return candidate

    # 그래도 못 찾으면 원본 그대로 반환 (이후 진단 메시지에서 알려줌)
    return p


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--images_dir", required=True, help="원본 이미지가 들어있는 폴더")
    parser.add_argument("--labels_dir", required=True, help="JSON 라벨이 들어있는 폴더")
    parser.add_argument("--output_dir", default="data/datasets", help="YOLO 데이터셋을 생성할 폴더")
    parser.add_argument("--val_ratio", type=float, default=0.2, help="검증 데이터 비율 (기본 0.2 = 20%%)")
    parser.add_argument("--seed", type=int, default=42, help="train/val 분리용 랜덤 시드")
    args = parser.parse_args()

    images_dir = resolve_dir(args.images_dir)
    labels_dir = resolve_dir(args.labels_dir)
    output_dir = Path(args.output_dir)

    # 진단 정보 출력 (경로 문제 디버깅용)
    print(f"[진단] labels_dir 원본 인자: {args.labels_dir!r}")
    print(f"[진단] labels_dir 존재 여부: {labels_dir.exists()}")
    print(f"[진단] labels_dir 절대경로: {labels_dir.resolve()}")
    if labels_dir.exists():
        entries = list(labels_dir.iterdir())
        print(f"[진단] labels_dir 안 항목 개수: {len(entries)}")
        if entries:
            print(f"[진단] 첫 항목 예시: {entries[0].name!r}")

    json_files = list(labels_dir.rglob("*.json"))
    print(f"JSON 파일 {len(json_files)}개 발견")

    if not json_files:
        print("JSON 파일을 찾지 못했습니다. --labels_dir 경로를 확인하세요.")
        return

    # 1단계: 모든 JSON을 먼저 읽어서 growth_stage 종류를 확정 (클래스 목록 만들기)
    parsed = []  # (img_path, growth_stage, boxes)
    skipped_no_image = 0
    skipped_no_annotation = 0
    skipped_no_stage = 0

    for jf in json_files:
        result = process_json_file(jf)
        if result is None:
            continue
        fname, image_id, fext, growth_stage, boxes = result

        if not boxes:
            skipped_no_annotation += 1
            continue

        if not growth_stage:
            skipped_no_stage += 1
            continue

        img_path = find_image_path(images_dir, fname, image_id, fext)
        if img_path is None:
            skipped_no_image += 1
            if skipped_no_image <= 3:
                print(f"  [매칭 실패 예시] fname={fname}, image_id={image_id}")
            continue

        parsed.append((img_path, growth_stage, boxes))

    # growth_stage 종류 확정 (알파벳/가나다 순 정렬로 항상 동일한 순서 보장)
    stage_names = sorted({p[1] for p in parsed})
    stage_to_id = {name: idx for idx, name in enumerate(stage_names)}

    print(f"\n발견된 성장 단계 종류 ({len(stage_names)}개): {stage_names}")
    for name in stage_names:
        count = sum(1 for p in parsed if p[1] == name)
        print(f"  - {name}: {count}개")

    # 2단계: (이미지경로, YOLO 라벨라인 리스트)로 변환
    pairs = []
    for img_path, growth_stage, boxes in parsed:
        class_id = stage_to_id[growth_stage]
        lines = [f"{class_id} {x_c:.6f} {y_c:.6f} {w:.6f} {h:.6f}" for x_c, y_c, w, h in boxes]
        pairs.append((img_path, lines))

    print(f"\n변환 가능한 쌍: {len(pairs)}개")
    print(f"  - 대상 카테고리('{TARGET_CATEGORY_NAME}') 라벨 없어서 제외: {skipped_no_annotation}개")
    print(f"  - growth_stage 정보 없어서 제외: {skipped_no_stage}개")
    print(f"  - 매칭되는 이미지 파일을 못 찾아서 제외: {skipped_no_image}개")

    if not pairs:
        print("변환할 데이터가 없습니다. 폴더 경로와 JSON 구조를 확인하세요.")
        return

    # 2단계: train/val 분리
    random.seed(args.seed)
    random.shuffle(pairs)
    val_count = max(1, int(len(pairs) * args.val_ratio))
    val_pairs = pairs[:val_count]
    train_pairs = pairs[val_count:]

    print(f"Train: {len(train_pairs)}개 / Val: {len(val_pairs)}개")

    # 3단계: 폴더 생성 및 파일 복사
    for split, split_pairs in [("train", train_pairs), ("val", val_pairs)]:
        img_out = output_dir / "images" / split
        label_out = output_dir / "labels" / split
        img_out.mkdir(parents=True, exist_ok=True)
        label_out.mkdir(parents=True, exist_ok=True)

        for img_path, lines in split_pairs:
            # 이미지 복사
            dst_img = img_out / img_path.name
            shutil.copy2(img_path, dst_img)

            # 라벨 txt 저장 (이미지와 같은 파일명, 확장자만 .txt)
            label_name = img_path.stem + ".txt"
            with open(label_out / label_name, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))

    # 4단계: data.yaml 생성 (성장 단계를 클래스로 사용)
    data_yaml_path = output_dir / "data.yaml"
    with open(data_yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {output_dir.resolve()}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n\n")
        f.write("names:\n")
        for idx, name in enumerate(stage_names):
            f.write(f"  {idx}: {name}\n")

    print("\n완료!")
    print(f"데이터셋 위치: {output_dir}")
    print(f"data.yaml 생성됨: {data_yaml_path}")
    print("\n이제 아래 명령으로 학습을 시작할 수 있습니다:")
    print("  python src/train.py")


if __name__ == "__main__":
    main()