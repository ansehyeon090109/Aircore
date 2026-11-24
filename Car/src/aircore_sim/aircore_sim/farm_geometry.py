"""Farm map geometry shared by the Gazebo world generator, the launch file and the 2D map script.

Single source of truth for the scale / floor / wall defaults (edit only here):
  MAP_SCALE, FLOOR_WIDTH_X, WALL_HEIGHT, WALL_THICKNESS

World frame convention (after scaling):
  - floor = STL plate + optional box extension on the +x side (blocks keep their size/position on the STL)
  - floor center (x, y) -> world origin (0, 0)
  - floor top surface -> z = 0
"""
import os
import shutil
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Defaults: change them only here (launch args / script args default to these)
# ---------------------------------------------------------------------------
MAP_SCALE = 0.001        # STL unit -> m (원본 단위 mm 확정, STL 바닥판 0.8 m x 1.2 m)
FLOOR_WIDTH_X = 1.6      # [m] 전체 바닥 x폭. STL 바닥판(0.8 m)보다 넓으면 차이만큼 +x쪽에 바닥을 덧붙임
WALL_HEIGHT = 0.10       # [m] 임시값 (미측정, 블록 최대 높이 0.05 m보다 높게)
WALL_THICKNESS = 0.01    # [m] 임시값 (미측정)

# Sensor (lidar) rendering engine. 'ogre' (1.x): with 'ogre2' the server's sensor rendering plus the GUI
# (also ogre2) ran out of GPU memory on the Jetson Orin Nano (NvMapMemAlloc error 12, black 3D view).
SENSOR_RENDER_ENGINE = 'ogre'

STL_NAME = 'MAP_3.stl'
MODEL_NAME = 'farm_map'
WORLD_NAME = 'farm'


@dataclass
class Box:
    """Axis-aligned box in world meters (z measured from the plate top surface)."""
    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float

    @property
    def size(self) -> Tuple[float, float, float]:
        return (self.xmax - self.xmin, self.ymax - self.ymin, self.zmax - self.zmin)

    @property
    def center(self) -> Tuple[float, float, float]:
        return ((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2, (self.zmin + self.zmax) / 2)

    def overlaps_xy(self, other: 'Box', eps: float = 1e-9) -> bool:
        return (self.xmin < other.xmax - eps and other.xmin < self.xmax - eps
                and self.ymin < other.ymax - eps and other.ymin < self.ymax - eps)


@dataclass
class FarmGeometry:
    scale: float
    center_xy: Tuple[float, float]   # STL bbox center, STL units
    plate_top_z: float               # STL units
    n_triangles: int
    ext_x: float                     # [m] floor extension added on the +x side (0 = none)
    stl_plate: Box                   # world m, the plate inside the STL
    floor_ext: Optional[Box]         # world m, extension box (None if ext_x == 0)
    plate: Box                       # world m, whole floor (stl_plate + floor_ext)
    blocks: List[Box]                # world m

    @property
    def mesh_offset(self) -> Tuple[float, float, float]:
        """Pose offset (m) of the scaled STL so the floor center -> origin, plate top -> z=0."""
        s = self.scale
        return (-self.center_xy[0] * s - self.ext_x / 2, -self.center_xy[1] * s, -self.plate_top_z * s)


# ---------------------------------------------------------------------------
# STL
# ---------------------------------------------------------------------------
def read_binary_stl(path: str) -> np.ndarray:
    """Return triangles as float64 array (n, 3, 3). Binary STL only."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 84:
        raise ValueError(f'{path}: too small for an STL file')
    n = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + 50 * n:
        raise ValueError(f'{path}: not a binary STL (size {len(data)} != 84 + 50*{n})')
    dt = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(data, dtype=dt, count=n, offset=84)['v'].astype(np.float64)


def load_farm_geometry(stl_path: str, scale: float = MAP_SCALE,
                       floor_width_x: float = FLOOR_WIDTH_X) -> FarmGeometry:
    """Extract the floor plate and the axis-aligned blocks standing on it from the STL.

    If floor_width_x is wider than the scaled STL plate, the floor is extended on the +x side.
    """
    if not scale > 0.0:
        raise ValueError(f'map_scale must be > 0 (got {scale})')
    tris = read_binary_stl(stl_path)
    v = tris.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    tol = 1e-6 * float(np.max(hi - lo))

    stl_width = float(hi[0] - lo[0]) * scale
    ext = round(floor_width_x - stl_width, 9)
    if ext < 0:
        raise ValueError(f'floor_width_x {floor_width_x} m < STL plate width {stl_width:.4f} m '
                         '(바닥은 넓히기만 가능)')

    # Plate top = highest vertex at the bbox corner (xmin, ymin); only the plate reaches it.
    corner = (np.abs(v[:, 0] - lo[0]) < tol) & (np.abs(v[:, 1] - lo[1]) < tol)
    if not corner.any():
        raise ValueError('no vertex at the bbox corner: floor plate not found')
    plate_top = float(v[corner, 2].max())
    cx, cy = float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)

    # Blocks = triangles rising above the plate top, grouped by shared vertices.
    above = tris[:, :, 2].max(axis=1) > plate_top + tol
    block_tris = tris[above]
    keys = [tuple(k) for k in np.round(block_tris.reshape(-1, 3) / tol).astype(np.int64)]
    parent = list(range(len(keys)))
    first = {}

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, k in enumerate(keys):          # same position -> same set
        if k in first:
            parent[find(i)] = find(first[k])
        else:
            first[k] = i
    for t in range(len(block_tris)):      # same triangle -> same set
        a = find(3 * t)
        for j in (1, 2):
            parent[find(3 * t + j)] = a
    groups = {}
    for i in range(len(keys)):
        groups.setdefault(find(i), []).append(i)

    s = scale

    def to_world(name, bmin, bmax) -> Box:   # STL center -> (-ext/2, 0) so the whole floor is centered
        return Box(name,
                   (bmin[0] - cx) * s - ext / 2, (bmax[0] - cx) * s - ext / 2,
                   (bmin[1] - cy) * s, (bmax[1] - cy) * s,
                   (bmin[2] - plate_top) * s, (bmax[2] - plate_top) * s)

    pts = block_tris.reshape(-1, 3)
    raw_blocks = []
    for idx in groups.values():
        p = pts[idx]
        bmin, bmax = p.min(axis=0), p.max(axis=0)
        on_corner = (np.minimum(np.abs(p - bmin), np.abs(p - bmax)) < tol).all()
        if not on_corner:
            raise ValueError(f'block near x={bmin[0]:.2f}, y={bmin[1]:.2f} is not an axis-aligned box')
        raw_blocks.append((bmin, bmax))
    raw_blocks.sort(key=lambda b: (b[0][0], b[0][1]))
    blocks = [to_world(f'block_{i}', b[0], b[1]) for i, b in enumerate(raw_blocks)]

    sp = to_world('stl_plate', lo, np.array([hi[0], hi[1], plate_top]))
    floor_ext = (Box('floor_extension', sp.xmax, sp.xmax + ext, sp.ymin, sp.ymax, sp.zmin, sp.zmax)
                 if ext > 0 else None)
    plate = Box('plate', sp.xmin, sp.xmax + ext, sp.ymin, sp.ymax, sp.zmin, sp.zmax)
    return FarmGeometry(scale, (cx, cy), plate_top, len(tris), ext, sp, floor_ext, plate, blocks)


# ---------------------------------------------------------------------------
# Walls
# ---------------------------------------------------------------------------
def wall_boxes(plate: Box, height: float, thickness: float) -> List[Box]:
    """4 walls standing on the plate; outer faces flush with the plate edges (no overhang)."""
    t, h = thickness, height
    return [
        Box('wall_north', plate.xmin, plate.xmax, plate.ymax - t, plate.ymax, 0.0, h),
        Box('wall_south', plate.xmin, plate.xmax, plate.ymin, plate.ymin + t, 0.0, h),
        Box('wall_east', plate.xmax - t, plate.xmax, plate.ymin + t, plate.ymax - t, 0.0, h),
        Box('wall_west', plate.xmin, plate.xmin + t, plate.ymin + t, plate.ymax - t, 0.0, h),
    ]


def validate(geom: FarmGeometry, wall_height: float, wall_thickness: float) -> List[str]:
    """Raise ValueError for impossible values, return warnings for suspicious ones."""
    w, d, _ = geom.plate.size
    if not wall_height > 0.0:
        raise ValueError(f'wall_height must be > 0 (got {wall_height})')
    if not wall_thickness > 0.0:
        raise ValueError(f'wall_thickness must be > 0 (got {wall_thickness})')
    if 2 * wall_thickness >= min(w, d):
        raise ValueError(f'wall_thickness {wall_thickness} m too large for plate {w:.3f} x {d:.3f} m')
    warnings = []
    max_block = max((b.zmax for b in geom.blocks), default=0.0)
    if wall_height <= max_block:
        warnings.append(f'wall_height {wall_height} m <= 블록 최대 높이 {max_block:.3f} m')
    for wall in wall_boxes(geom.plate, wall_height, wall_thickness):
        for b in geom.blocks:
            if wall.overlaps_xy(b):
                warnings.append(f'{wall.name}이(가) {b.name}와 겹칩니다 (wall_thickness 확인)')
    return warnings


# ---------------------------------------------------------------------------
# SDF generation (Gazebo Fortress / sdformat 12)
# ---------------------------------------------------------------------------
def _f(x: float) -> str:
    x = round(float(x), 9)
    return '0' if x == 0 else format(x, '.9g')


def _vec(*xs: float) -> str:
    return ' '.join(_f(x) for x in xs)


def model_sdf(geom: FarmGeometry, wall_height: float, wall_thickness: float) -> str:
    s = geom.scale
    ox, oy, oz = geom.mesh_offset
    mesh = (f'<mesh><uri>model://{MODEL_NAME}/meshes/{STL_NAME}</uri>'
            f'<scale>{_vec(s, s, s)}</scale></mesh>')
    wall_xml = []
    for wbox in wall_boxes(geom.plate, wall_height, wall_thickness):
        pose = _vec(*wbox.center, 0, 0, 0)
        size = _vec(*wbox.size)
        wall_xml.append(f"""      <collision name="{wbox.name}_collision">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="{wbox.name}_visual">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material><ambient>0.55 0.6 0.75 1</ambient><diffuse>0.55 0.6 0.75 1</diffuse></material>
      </visual>""")
    walls = '\n'.join(wall_xml)
    ext_xml = ''
    if geom.floor_ext is not None:
        e = geom.floor_ext
        pose, size = _vec(*e.center, 0, 0, 0), _vec(*e.size)
        ext_xml = f"""
    <link name="floor_extension">
      <!-- floor added on the +x side of the STL plate ({_f(geom.ext_x)} m), same thickness, top at z=0 -->
      <collision name="collision">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material><ambient>0.62 0.58 0.5 1</ambient><diffuse>0.62 0.58 0.5 1</diffuse></material>
      </visual>
    </link>"""
    p = geom.plate
    return f"""<?xml version="1.0"?>
<!-- Generated by aircore_sim/farm_geometry.py - do not edit by hand.
     map_scale={_f(s)} floor {_f(p.size[0])} x {_f(p.size[1])} m (STL plate + {_f(geom.ext_x)} m on +x)
     wall_height={_f(wall_height)} m wall_thickness={_f(wall_thickness)} m (wall values are temporary) -->
<sdf version="1.8">
  <model name="{MODEL_NAME}">
    <static>true</static>
    <link name="map_mesh">
      <!-- STL center ({_vec(*geom.center_xy)}) -> x={_f(-geom.ext_x / 2)}, y=0 so the whole floor is centered;
           plate top (z={_f(geom.plate_top_z)}) -> z=0, after scale -->
      <pose>{_vec(ox, oy, oz, 0, 0, 0)}</pose>
      <collision name="collision">
        <geometry>{mesh}</geometry>
      </collision>
      <visual name="visual">
        <geometry>{mesh}</geometry>
        <material><ambient>0.62 0.58 0.5 1</ambient><diffuse>0.62 0.58 0.5 1</diffuse></material>
      </visual>
    </link>{ext_xml}
    <link name="walls">
{walls}
    </link>
  </model>
</sdf>
"""


def model_config() -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author><name>aircore</name></author>
  <description>Smart farm map ({STL_NAME}, mm) with 4 perimeter walls</description>
</model>
"""


def world_sdf(geom: FarmGeometry, includes=(), sensors: bool = False) -> str:
    """includes: [(model_name, (x, y, z, roll, pitch, yaw))] added after farm_map.
    sensors: add the Sensors system (needed by gpu_lidar)."""
    ground_z = geom.plate.zmin  # plate bottom, so the ground does not z-fight with the plate top
    sensors_xml = f"""
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>{SENSOR_RENDER_ENGINE}</render_engine>
    </plugin>""" if sensors else ''
    include_xml = ''.join(f"""

    <include>
      <uri>model://{name}</uri>
      <name>{name}</name>
      <pose>{_vec(*pose)}</pose>
    </include>""" for name, pose in includes)
    return f"""<?xml version="1.0"?>
<!-- Generated by aircore_sim/farm_geometry.py - do not edit by hand. Needs IGN_GAZEBO_RESOURCE_PATH=<aircore_sim>/models -->
<sdf version="1.8">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>{sensors_xml}

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>{_vec(0, 0, ground_z, 0, 0, 0)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://{MODEL_NAME}</uri>
      <name>{MODEL_NAME}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>{include_xml}
  </world>
</sdf>
"""


@dataclass
class WorldFiles:
    models_dir: str
    model_sdf: str
    world_sdf: str


def write_world_files(out_root: str, geom: FarmGeometry, wall_height: float,
                      wall_thickness: float, stl_src: str, includes=(), sensors: bool = False) -> WorldFiles:
    """Write <out_root>/models/farm_map/{model.sdf,model.config,meshes/STL} and <out_root>/worlds/farm.sdf."""
    models_dir = os.path.join(out_root, 'models')
    model_dir = os.path.join(models_dir, MODEL_NAME)
    mesh_dir = os.path.join(model_dir, 'meshes')
    world_dir = os.path.join(out_root, 'worlds')
    os.makedirs(mesh_dir, exist_ok=True)
    os.makedirs(world_dir, exist_ok=True)

    mesh_dst = os.path.join(mesh_dir, STL_NAME)
    if not (os.path.exists(mesh_dst) and os.path.samefile(stl_src, mesh_dst)):
        shutil.copyfile(stl_src, mesh_dst)

    files = WorldFiles(models_dir, os.path.join(model_dir, 'model.sdf'),
                       os.path.join(world_dir, f'{WORLD_NAME}.sdf'))
    _write(files.model_sdf, model_sdf(geom, wall_height, wall_thickness))
    _write(os.path.join(model_dir, 'model.config'), model_config())
    _write(files.world_sdf, world_sdf(geom, includes, sensors))
    return files


def _write(path: str, text: str) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def describe(geom: FarmGeometry) -> List[str]:
    """Human-readable summary lines (world meters)."""
    p, sp = geom.plate, geom.stl_plate
    lines = [
        f'STL triangles={geom.n_triangles} center=({_vec(*geom.center_xy)}) plate_top_z={_f(geom.plate_top_z)} '
        f'(STL units), map_scale={_f(geom.scale)}',
        f'floor: x {p.xmin:+.4f}~{p.xmax:+.4f}  y {p.ymin:+.4f}~{p.ymax:+.4f}  '
        f'size {p.size[0]:.4f} x {p.size[1]:.4f} m, thickness {p.size[2]:.4f} m (top z=0)',
        f'  stl_plate: x {sp.xmin:+.4f}~{sp.xmax:+.4f} ({sp.size[0]:.4f} m)'
        + (f', floor_extension: x {geom.floor_ext.xmin:+.4f}~{geom.floor_ext.xmax:+.4f} ({geom.ext_x:.4f} m)'
           if geom.floor_ext else ', no extension'),
    ]
    for b in geom.blocks:
        lines.append(f'{b.name}: x {b.xmin:+.4f}~{b.xmax:+.4f}  y {b.ymin:+.4f}~{b.ymax:+.4f}  '
                     f'height {b.zmax:.4f} m')
    return lines
