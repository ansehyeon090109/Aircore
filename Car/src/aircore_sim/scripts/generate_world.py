#!/usr/bin/env python3
"""Regenerate models/farm_map/model.sdf, model.config and worlds/farm.sdf from the STL.

Defaults come from aircore_sim/farm_geometry.py (MAP_SCALE, WALL_HEIGHT, WALL_THICKNESS).
sim_world.launch.py regenerates its own copy at launch time; this script refreshes the
files kept in the package (used for a direct `ign gazebo worlds/farm.sdf`).
"""
import argparse
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
if (PKG_ROOT / 'aircore_sim' / 'farm_geometry.py').exists():  # run from source / symlink install
    sys.path.insert(0, str(PKG_ROOT))
from aircore_sim import farm_geometry as fg  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stl', default=str(PKG_ROOT / 'models' / fg.MODEL_NAME / 'meshes' / fg.STL_NAME))
    ap.add_argument('--map-scale', type=float, default=fg.MAP_SCALE)
    ap.add_argument('--floor-width-x', type=float, default=fg.FLOOR_WIDTH_X,
                    help='[m] 전체 바닥 x폭 (STL 바닥판보다 넓으면 +x쪽으로 확장)')
    ap.add_argument('--wall-height', type=float, default=fg.WALL_HEIGHT, help='[m] (임시값)')
    ap.add_argument('--wall-thickness', type=float, default=fg.WALL_THICKNESS, help='[m] (임시값)')
    ap.add_argument('--out-root', default=str(PKG_ROOT), help='models/ 와 worlds/ 를 쓸 디렉터리')
    a = ap.parse_args()

    geom = fg.load_farm_geometry(a.stl, a.map_scale, a.floor_width_x)
    for w in fg.validate(geom, a.wall_height, a.wall_thickness):
        print(f'[WARN] {w}')
    print(f'[WARN] 벽 높이/두께는 임시값입니다: wall_height={a.wall_height} m, '
          f'wall_thickness={a.wall_thickness} m')
    files = fg.write_world_files(a.out_root, geom, a.wall_height, a.wall_thickness, a.stl)
    for line in fg.describe(geom):
        print(line)
    print(f'mesh pose offset: {fg._vec(*geom.mesh_offset)} m')
    print(f'wrote {files.model_sdf}\nwrote {files.world_sdf}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
