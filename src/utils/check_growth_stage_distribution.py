"""
라벨링데이터(JSON) 전체를 스캔해서, crops_id(=원천데이터 폴더 대응)별로
growth_stage가 어떻게 분포하는지 확인하는 진단 스크립트.

이미지 없이 JSON만으로 실행 가능합니다.
다운로드해야 할 원천데이터 폴더를 파악하는 용도로 사용하세요.

사용법:
    python src/utils/check_growth_stage_distribution.py \
        --labels_dir "JSON 파일이 들어있는 폴더 경로 (라벨링데이터)"
"""

import argparse
import json
import unicodedata
from collections import defaultdict
from pathlib import Path


def resolve_dir(path_str: str) -> Path:
    p = Path(path_str)
    if p.exists():
        return p
    for form in ("NFC", "NFD"):
        candidate = Path(unicodedata.normalize(form, path_str))
        if candidate.exists():
            return candidate
    return p


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--labels_dir", required=True, help="JSON 라벨이 들어있는 폴더 (라벨링데이터)")
    args = parser.parse_args()

    labels_dir = resolve_dir(args.labels_dir)
    json_files = list(labels_dir.rglob("*.json"))
    print(f"JSON 파일 {len(json_files)}개 발견\n")

    # crops_id(예: C11_L01_01) -> growth_stage -> 개수
    dist = defaultdict(lambda: defaultdict(int))
    # 전체 growth_stage 분포
    overall = defaultdict(int)
    # 어떤 상위 폴더(예: TL_3.상추1)에 있었는지도 기록
    folder_stage = defaultdict(lambda: defaultdict(int))

    for jf in json_files:
        try:
            with open(jf, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            continue

        img_info = data.get("images", {})
        crops_id = img_info.get("crops_id", "unknown")
        growth_stage = img_info.get("growth_stage", "없음")

        dist[crops_id][growth_stage] += 1
        overall[growth_stage] += 1

        # JSON 파일이 들어있던 상위 폴더명 (TL_3.상추N)
        folder_name = jf.parent.name
        folder_stage[folder_name][growth_stage] += 1

    print("=== 전체 성장 단계 분포 ===")
    for stage, count in sorted(overall.items()):
        print(f"  {stage}: {count}개")

    print("\n=== 폴더별 성장 단계 분포 (라벨 기준) ===")
    for folder in sorted(folder_stage.keys()):
        stages = folder_stage[folder]
        stage_summary = ", ".join(f"{s}:{c}" for s, c in sorted(stages.items()))
        print(f"  {folder} -> {stage_summary}")

    print("\n위 목록을 보고, 다양한 성장 단계를 포함하려면 어떤 원천데이터 폴더를")
    print("추가로 다운로드해야 하는지 확인하세요 (라벨 폴더명 TL_3.상추N ↔ 이미지 폴더명 TS_3.상추N).")


if __name__ == "__main__":
    main()