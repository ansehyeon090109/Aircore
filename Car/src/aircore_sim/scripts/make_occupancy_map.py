#!/usr/bin/env python3
"""Build a Nav2 occupancy map (PGM + YAML) straight from the STL boxes and wall parameters (no SLAM).

Cell rules:
  - obstacle (block / wall) whose footprint touches the cell and zmin <= scan_height < zmax -> occupied (0)
  - cell not fully inside the floor plate                                                -> occupied (0)
  - everything else                                                                      -> free (254)
Image row 0 is the top (max y); origin in the YAML is the lower-left corner in world coordinates.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if (PKG_ROOT / 'aircore_sim' / 'farm_geometry.py').exists():  # run from source / symlink install
    sys.path.insert(0, str(PKG_ROOT))
from aircore_sim import farm_geometry as fg  # noqa: E402

SCAN_HEIGHT = 0.02   # [m] 임시값: 라이다 스캔 평면 높이 (바닥판 윗면 기준, 미측정)
RESOLUTION = 0.01    # [m/cell]
OCCUPIED, FREE = 0, 254
EPS = 1e-6           # in cells; absorbs float error on grid-aligned edges


def cell_range(lo: float, hi: float, origin: float, res: float, n: int, inner: bool):
    """Index range [i0, i1) of cells touching [lo, hi] (inner=False) or fully inside it (inner=True)."""
    a, b = (lo - origin) / res, (hi - origin) / res
    if inner:
        i0, i1 = math.ceil(a - EPS), math.floor(b + EPS)
    else:
        i0, i1 = math.floor(a + EPS), math.ceil(b - EPS)
    return max(0, i0), min(n, i1)


def build_grid(geom: fg.FarmGeometry, walls, res: float, scan_height: float, margin: float):
    p = geom.plate
    ox, oy = p.xmin - margin, p.ymin - margin
    w = math.ceil(round((p.xmax - p.xmin + 2 * margin) / res, 6))
    h = math.ceil(round((p.ymax - p.ymin + 2 * margin) / res, 6))

    occ = np.ones((h, w), dtype=bool)            # [row = y index from bottom, col = x index]
    x0, x1 = cell_range(p.xmin, p.xmax, ox, res, w, inner=True)
    y0, y1 = cell_range(p.ymin, p.ymax, oy, res, h, inner=True)
    occ[y0:y1, x0:x1] = False                    # inside the plate -> free for now

    used, skipped = [], []
    for box in list(geom.blocks) + list(walls):
        if box.zmin <= scan_height < box.zmax:
            x0, x1 = cell_range(box.xmin, box.xmax, ox, res, w, inner=False)
            y0, y1 = cell_range(box.ymin, box.ymax, oy, res, h, inner=False)
            occ[y0:y1, x0:x1] = True
            used.append(box)
        else:
            skipped.append(box)
    return occ, (ox, oy), used, skipped


def write_pgm(path: Path, occ: np.ndarray) -> None:
    img = np.where(occ, OCCUPIED, FREE).astype(np.uint8)[::-1]   # flip: top row = max y
    with open(path, 'wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode('ascii'))
        f.write(img.tobytes())


def write_yaml(path: Path, image_name: str, res: float, origin) -> None:
    path.write_text(
        f'image: {image_name}\n'
        f'mode: trinary\n'
        f'resolution: {fg._f(res)}\n'
        f'origin: [{fg._f(origin[0])}, {fg._f(origin[1])}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.25\n', encoding='ascii')


def write_preview(path: Path, occ: np.ndarray, geom, walls, used, origin, res: float) -> bool:
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print('[WARN] python3-pil 없음: 미리보기 PNG 생략')
        return False
    h, w = occ.shape
    k = max(1, 600 // max(w, h))
    rgb = np.full((h, w, 3), 255, dtype=np.uint8)
    rgb[occ] = (30, 30, 30)                                  # occupied (outside plate etc.)
    ox, oy = origin
    for box in used:                                         # colour by obstacle type
        colour = (70, 110, 200) if box.name.startswith('wall') else (60, 150, 60)
        x0, x1 = cell_range(box.xmin, box.xmax, ox, res, w, inner=False)
        y0, y1 = cell_range(box.ymin, box.ymax, oy, res, h, inner=False)
        rgb[y0:y1, x0:x1] = colour
    img = Image.fromarray(rgb[::-1]).resize((w * k, h * k), Image.NEAREST)
    d = ImageDraw.Draw(img)
    for i in range(0, w + 1, max(1, round(0.1 / res))):      # 0.1 m grid
        d.line([(i * k, 0), (i * k, h * k)], fill=(205, 205, 205))
    for j in range(0, h + 1, max(1, round(0.1 / res))):
        d.line([(0, (h - j) * k), (w * k, (h - j) * k)], fill=(205, 205, 205))
    cx, cy = (0 - ox) / res * k, (h - (0 - oy) / res) * k    # world origin
    d.line([(cx - 8, cy), (cx + 8, cy)], fill=(220, 30, 30), width=2)
    d.line([(cx, cy - 8), (cx, cy + 8)], fill=(220, 30, 30), width=2)
    img.save(path)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stl', default=str(PKG_ROOT / 'models' / fg.MODEL_NAME / 'meshes' / fg.STL_NAME))
    ap.add_argument('--map-scale', type=float, default=fg.MAP_SCALE)
    ap.add_argument('--floor-width-x', type=float, default=fg.FLOOR_WIDTH_X,
                    help='[m] 전체 바닥 x폭 (STL 바닥판보다 넓으면 +x쪽으로 확장)')
    ap.add_argument('--wall-height', type=float, default=fg.WALL_HEIGHT, help='[m] (임시값)')
    ap.add_argument('--wall-thickness', type=float, default=fg.WALL_THICKNESS, help='[m] (임시값)')
    ap.add_argument('--resolution', type=float, default=RESOLUTION, help='[m/cell]')
    ap.add_argument('--scan-height', type=float, default=SCAN_HEIGHT, help='[m] 라이다 평면 높이 (임시값)')
    ap.add_argument('--margin', type=float, default=0.0, help='[m] 바닥판 밖 여백 (점유로 채움)')
    ap.add_argument('--out-dir', default=str(PKG_ROOT / 'maps'))
    ap.add_argument('--name', default='farm')
    ap.add_argument('--no-preview', action='store_true')
    a = ap.parse_args()

    if not a.resolution > 0 or a.margin < 0:
        ap.error('resolution must be > 0 and margin >= 0')
    if a.scan_height == SCAN_HEIGHT:
        print(f'[WARN] scan_height={SCAN_HEIGHT} m 는 임시값입니다 (라이다 장착 높이 실측 후 --scan-height로 지정)')
    print(f'[WARN] 벽 높이/두께는 임시값입니다: wall_height={a.wall_height} m, wall_thickness={a.wall_thickness} m')

    geom = fg.load_farm_geometry(a.stl, a.map_scale, a.floor_width_x)
    for w in fg.validate(geom, a.wall_height, a.wall_thickness):
        print(f'[WARN] {w}')
    walls = fg.wall_boxes(geom.plate, a.wall_height, a.wall_thickness)
    occ, origin, used, skipped = build_grid(geom, walls, a.resolution, a.scan_height, a.margin)

    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    pgm, yml, png = out / f'{a.name}.pgm', out / f'{a.name}.yaml', out / f'{a.name}_preview.png'
    write_pgm(pgm, occ)
    write_yaml(yml, pgm.name, a.resolution, origin)

    for line in fg.describe(geom):
        print(line)
    h, w = occ.shape
    print(f'scan_height={a.scan_height} m: occupied obstacles = {[b.name for b in used]}')
    if skipped:
        print(f'  ignored (not crossing scan plane): {[b.name for b in skipped]}')
    print(f'grid {w} x {h} cells @ {a.resolution} m = {w * a.resolution:.4f} x {h * a.resolution:.4f} m, '
          f'origin ({origin[0]:+.4f}, {origin[1]:+.4f}), occupied {int(occ.sum())}/{occ.size}')
    print(f'wrote {pgm}\nwrote {yml}')
    if not a.no_preview and write_preview(png, occ, geom, walls, used, origin, a.resolution):
        print(f'wrote {png}  (black=outside plate, blue=wall, green=block, red cross=world origin, grid 0.1 m)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
