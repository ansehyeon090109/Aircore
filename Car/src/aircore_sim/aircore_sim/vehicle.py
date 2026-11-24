"""Aircore vehicle (aircore_description/urdf/aircore.urdf) -> Gazebo Fortress model.

The URDF is converted with Fortress's sdformat (`ign sdf -p`, package:// -> model://) and only
Fortress-specific pieces are added / fixed here (the URDF itself is not modified):
  - DiffDrive + JointStatePublisher systems with Fortress plugin names
  - lidar frame id: Harmonic <gz_frame_id> -> Fortress <ignition_frame_id>

wheel_separation / wheel_radius for the sim DiffDrive are read from the URDF geometry (CAD values),
they are NOT measured values and must not be copied into aircore_base's odometry parameters.
"""
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from aircore_sim.farm_geometry import Box

MODEL_NAME = 'aircore'
LEFT_JOINT = 'left_wheel_joint'
RIGHT_JOINT = 'right_wheel_joint'
BASE_FRAME = 'base_footprint'

# Default spawn pose: open floor right of the small blocks (edit only here).
SPAWN_X = 0.2
SPAWN_Y = 0.0
SPAWN_YAW = 0.0


@dataclass
class Vehicle:
    sdf: str
    wheel_separation: float   # [m] from URDF joints
    wheel_radius: float       # [m] from URDF wheel collision
    footprint_radius: float   # [m] conservative xy radius of all collisions around base_footprint


# ---------------------------------------------------------------------------
# URDF geometry
# ---------------------------------------------------------------------------
def _origin(elem) -> np.ndarray:
    m = np.eye(4)
    o = elem.find('origin') if elem is not None else None
    if o is None:
        return m
    r, p, y = map(float, (o.get('rpy') or '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = list(map(float, (o.get('xyz') or '0 0 0').split()))
    return m


def _link_poses(robot) -> dict:
    """Pose of every link relative to the root link (joint zero positions)."""
    parent = {j.find('child').get('link'): (j.find('parent').get('link'), _origin(j))
              for j in robot.findall('joint')}
    poses = {}
    for link in robot.findall('link'):
        name, m = link.get('name'), np.eye(4)
        while name in parent:
            name, jm = parent[name][0], parent[name][1]
            m = jm @ m
        poses[link.get('name')] = m
    return poses


def wheel_params(robot) -> Tuple[float, float]:
    joints = {j.get('name'): j for j in robot.findall('joint')}
    for name in (LEFT_JOINT, RIGHT_JOINT):
        if name not in joints:
            raise ValueError(f'URDF has no joint {name}')
    poses = _link_poses(robot)
    yl = poses[joints[LEFT_JOINT].find('child').get('link')][1, 3]
    yr = poses[joints[RIGHT_JOINT].find('child').get('link')][1, 3]
    radii = []
    for name in (LEFT_JOINT, RIGHT_JOINT):
        link = robot.find(f"link[@name='{joints[name].find('child').get('link')}']")
        cyl = link.find('collision/geometry/cylinder')
        if cyl is None:
            raise ValueError(f'{link.get("name")}: wheel collision is not a cylinder')
        radii.append(float(cyl.get('radius')))
    if abs(radii[0] - radii[1]) > 1e-6:
        raise ValueError(f'left/right wheel radius differ: {radii}')
    return abs(yl - yr), radii[0]


def footprint_radius(robot) -> float:
    """Conservative xy radius (around the root link origin) covering every collision shape."""
    poses = _link_poses(robot)
    r_max = 0.0
    for link in robot.findall('link'):
        for col in link.findall('collision'):
            m = poses[link.get('name')] @ _origin(col)
            g = col.find('geometry')
            shape = g[0]
            upright = abs(m[2, 2] - 1.0) < 1e-6          # local z still vertical (yaw-only rotation)
            if shape.tag == 'box':
                sx, sy, sz = map(float, shape.get('size').split())
                r = 0.5 * (math.hypot(sx, sy) if upright else math.sqrt(sx * sx + sy * sy + sz * sz))
            elif shape.tag == 'cylinder':
                rad, half = float(shape.get('radius')), float(shape.get('length')) / 2
                r = rad if upright else math.hypot(rad, half)
            elif shape.tag == 'sphere':
                r = float(shape.get('radius'))
            else:  # mesh collision: not used by this robot, be loud instead of guessing
                raise ValueError(f'{link.get("name")}: unsupported collision {shape.tag}')
            r_max = max(r_max, math.hypot(m[0, 3], m[1, 3]) + r)
    return r_max


# ---------------------------------------------------------------------------
# SDF
# ---------------------------------------------------------------------------
def _sub(parent, tag: str, text=None, **attrib):
    e = ET.SubElement(parent, tag, attrib)
    if text is not None:
        e.text = str(text)
    return e


def build_vehicle(urdf_path: str) -> Vehicle:
    robot = ET.parse(urdf_path).getroot()
    sep, radius = wheel_params(robot)
    fp = footprint_radius(robot)

    res = subprocess.run(['ign', 'sdf', '-p', urdf_path], capture_output=True, text=True)
    if res.returncode != 0 or not res.stdout.strip():
        raise RuntimeError(f'ign sdf -p failed ({res.returncode}): {res.stderr.strip()}')
    sdf = ET.fromstring(res.stdout)
    model = sdf.find('model')
    if model is None:
        raise RuntimeError('converted SDF has no <model>')
    model.set('name', MODEL_NAME)

    for sensor in model.iter('sensor'):            # Harmonic -> Fortress frame id
        frame = None
        for tag in ('gz_frame_id', 'frame_id'):
            e = sensor.find(tag)
            if e is not None:
                frame = frame or e.text
                sensor.remove(e)
        if frame and sensor.find('ignition_frame_id') is None:
            _sub(sensor, 'ignition_frame_id', frame)

    dd = _sub(model, 'plugin', filename='ignition-gazebo-diff-drive-system',
              name='ignition::gazebo::systems::DiffDrive')
    for tag, val in (('left_joint', LEFT_JOINT), ('right_joint', RIGHT_JOINT),
                     ('wheel_separation', f'{sep:.6g}'), ('wheel_radius', f'{radius:.6g}'),
                     ('odom_publish_frequency', 30), ('frame_id', 'odom'), ('child_frame_id', BASE_FRAME)):
        _sub(dd, tag, val)
    _sub(model, 'plugin', filename='ignition-gazebo-joint-state-publisher-system',
         name='ignition::gazebo::systems::JointStatePublisher')

    ET.indent(sdf, space='  ')
    text = ('<?xml version="1.0"?>\n'
            f'<!-- Generated by aircore_sim/vehicle.py from {os.path.basename(urdf_path)} (ign sdf -p) - '
            'do not edit by hand.\n'
            f'     DiffDrive wheel_separation={sep:.6g} m wheel_radius={radius:.6g} m are URDF (CAD) values, '
            'not measured. -->\n'
            + ET.tostring(sdf, encoding='unicode') + '\n')
    return Vehicle(text, sep, radius, fp)


def write_vehicle_model(models_dir: str, vehicle: Vehicle) -> str:
    model_dir = os.path.join(models_dir, MODEL_NAME)
    os.makedirs(model_dir, exist_ok=True)
    path = os.path.join(model_dir, 'model.sdf')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(vehicle.sdf)
    with open(os.path.join(model_dir, 'model.config'), 'w', encoding='utf-8') as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>aircore</name></author>
  <description>aircore diff-drive robot (from aircore_description/urdf/aircore.urdf) for Gazebo Fortress</description>
</model>
""")
    return path


def check_spawn(floor: Box, obstacles: List[Box], x: float, y: float, radius: float) -> List[str]:
    """Warn if the vehicle footprint circle leaves the floor or touches an obstacle."""
    warnings = []
    if not (floor.xmin + radius <= x <= floor.xmax - radius and floor.ymin + radius <= y <= floor.ymax - radius):
        warnings.append(f'spawn ({x}, {y}) r={radius:.3f} m: 차량이 바닥 밖으로 나갑니다')
    for b in obstacles:
        dx = max(b.xmin - x, 0.0, x - b.xmax)
        dy = max(b.ymin - y, 0.0, y - b.ymax)
        if math.hypot(dx, dy) < radius:
            warnings.append(f'spawn ({x}, {y}) r={radius:.3f} m: {b.name}와 겹칩니다')
    return warnings
