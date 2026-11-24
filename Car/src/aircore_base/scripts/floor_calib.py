#!/usr/bin/env python3
"""Step 2 floor calibration: drive a bounded straight line or in-place turn through /cmd_vel,
then compare /odom with the real motion (LiDAR scan matching, tape measure / eye).

Needs bringup running (base_controller + odometry; LiDAR + robot_state_publisher for the
LiDAR measurement and the obstacle stop). Publishes /cmd_vel only after the plan is printed
and 'yes' is typed. Stops on: target reached (by /odom), time limit, no movement, stale /odom,
stale /scan, obstacle (LiDAR), Ctrl+C. base_controller itself stops 0.5 s after /cmd_vel stops.

  floor_calib.py straight --distance 1.0 --speed 0.5    # -> wheel_radius
  floor_calib.py rotate --angle 360 --turn 4.6          # -> wheel_separation

Do the straight test first: /odom yaw also scales with wheel_radius. The script only prints
suggested values; apply them with apply_measurements.py and restart bringup.
"""
import argparse
import collections
import json
import math
import os
import signal
import sys
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformListener

BASE_FRAME = 'base_footprint'
SEND_HZ = 20.0
ODOM_STALE_S = 0.5
SCAN_STALE_S = 0.5
STALL_S = 1.5                 # rolling window: must gain STALL_MIN_* in every STALL_S, otherwise stop
STALL_MIN_DIST = 0.03         # [m]
STALL_MIN_ANGLE = math.radians(5.0)
STRAIGHT_MAX_YAW = math.radians(30.0)  # straight: stop if heading drifts more than this
STOP_REPEAT = 10
SETTLE_S = 1.5                # coast + let /odom and /scan catch up before measuring
SCANS_PER_CAPTURE = 3

# Robot footprint (aircore.urdf base box 0.176 x 0.225 m, wheels at y=+-0.1085).
SELF_BOX = (0.12, 0.14)       # |x|, |y| in base_footprint: LiDAR points here are the robot itself
FRONT_X = 0.10                # front edge
CORRIDOR_HALF_W = 0.16        # straight: watch this half-width ahead
STRAIGHT_STOP_GAP = 0.25      # straight: stop if something is closer than this to the front edge
STOP_GAP_PER_SPEED = 0.4      # + this many metres per commanded m/s (reaction + coast)
ROTATE_CLEAR_R = 0.22         # rotate: nothing may be within this radius (half diagonal ~0.143)

LIMITS = {'speed': (0.05, 1.7), 'turn': (0.3, 16.0), 'duty': (1, 100), 'distance': (0.2, 2.0),
          'angle': (30.0, 720.0), 'max_time': (1.0, 15.0)}

BC_PARAMS = ['max_wheel_speed', 'max_duty', 'wheel_separation', 'invert_left', 'invert_right']
OD_PARAMS = ['wheel_radius', 'wheel_separation', 'ticks_per_rev_left', 'ticks_per_rev_right']
LOG_DIR = os.environ.get('AIRCORE_CALIB_LOG_DIR', os.path.expanduser('~/aircore_ws/calib_logs'))
SPINUP_S = 0.5                # ignored when computing steady wheel speed
SPEED_WIN_S = 0.25            # window for the speed-variation numbers


# ---------------- pure helpers (unit-testable) ----------------

def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rot(th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def round_half_away(x: float) -> int:
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def preview_duty(v: float, w: float, bc: dict) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Same conversion as base_controller: ((duty_l, duty_r) before invert, (sent_l, sent_r))."""
    L, vmax, dmax = bc['wheel_separation'], bc['max_wheel_speed'], bc['max_duty']
    out = []
    for vw in (v - w * L / 2.0, v + w * L / 2.0):
        d = round_half_away(vw / vmax * 100.0)
        out.append(max(-dmax, min(dmax, d)))
    sent = (-out[0] if bc['invert_left'] else out[0], -out[1] if bc['invert_right'] else out[1])
    return (out[0], out[1]), sent


def scan_to_points(msg: LaserScan, laser_pose: Tuple[float, float, float]) -> np.ndarray:
    """LaserScan -> (N, 2) points in base_footprint, robot body removed."""
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + np.arange(r.size) * msg.angle_increment
    ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= min(msg.range_max, 12.0))
    r, a = r[ok], a[ok]
    tx, ty, yaw = laser_pose
    pts = np.column_stack([r * np.cos(a), r * np.sin(a)]) @ rot(yaw).T + np.array([tx, ty])
    body = (np.abs(pts[:, 0]) < SELF_BOX[0]) & (np.abs(pts[:, 1]) < SELF_BOX[1])
    return pts[~body]


def front_gap(pts: np.ndarray) -> float:
    """Free distance ahead of the front edge inside the driving corridor."""
    sel = pts[(pts[:, 0] > FRONT_X) & (np.abs(pts[:, 1]) < CORRIDOR_HALF_W)]
    return float(sel[:, 0].min() - FRONT_X) if len(sel) else float('inf')


def nearest_radius(pts: np.ndarray) -> float:
    return float(np.hypot(pts[:, 0], pts[:, 1]).min()) if len(pts) else float('inf')


def coarse_rotation(src: np.ndarray, dst: np.ndarray, step_deg: float = 1.0) -> float:
    """Best pure rotation (about the base origin) mapping src onto dst, searched over 360 deg."""
    tree = cKDTree(dst)
    sub = src[::max(1, len(src) // 400)]
    best_th, best = 0.0, float('inf')
    for k in range(int(round(360.0 / step_deg))):
        th = math.radians(k * step_deg)
        d, _ = tree.query(sub @ rot(th).T)
        score = float(np.mean(np.minimum(d, 0.3)))
        if score < best:
            best_th, best = th, score
    return wrap(best_th)


def icp(src: np.ndarray, dst: np.ndarray, th: float, t: np.ndarray,
        iters: int = 50) -> Optional[Tuple[float, np.ndarray, float, float]]:
    """Trimmed point-to-point ICP. Returns (theta, t, inlier_fraction, rms) with
    dst ~= R(theta) @ src + t, i.e. the pose of the 'src' robot in the 'dst' robot frame."""
    if len(src) < 50 or len(dst) < 50:
        return None
    tree = cKDTree(dst)
    t = np.asarray(t, dtype=float)
    max_d = 0.5
    for _ in range(iters):
        d, j = tree.query(src @ rot(th).T + t)
        keep = d < max_d
        if keep.sum() < 30:
            return None
        keep &= d <= np.quantile(d[keep], 0.8)
        a, b = src[keep], dst[j[keep]]
        ma, mb = a.mean(0), b.mean(0)
        h = (a - ma).T @ (b - mb)
        th_new = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        t_new = mb - rot(th_new) @ ma
        done = abs(wrap(th_new - th)) < 1e-6 and np.linalg.norm(t_new - t) < 1e-6
        th, t = th_new, t_new
        max_d = max(0.05, max_d * 0.7)
        if done:
            break
    d, _ = tree.query(src @ rot(th).T + t)
    inl = d < 0.05
    rms = float(np.sqrt(np.mean(d[inl] ** 2))) if inl.any() else float('inf')
    return th, t, float(inl.mean()), rms


def resolve_turns(lidar_rad: float, odom_rad: float) -> float:
    """LiDAR gives the angle mod 360; pick the whole-turn count closest to odom."""
    k = round((odom_rad - lidar_rad) / (2.0 * math.pi))
    return lidar_rad + 2.0 * math.pi * k


def wheel_stats(samples, t_from: float, t_to: float, m_per_count: Tuple[float, float]) -> Optional[dict]:
    """samples: [(t, L, R)] direction-corrected cumulative counts (forward = +).
    Per wheel over [t_from, t_to]: distance, steady mean speed (spin-up skipped), the spread of
    SPEED_WIN_S-window speeds and the fraction of windows where the wheel was (nearly) stopped."""
    arr = np.array([s for s in samples if t_from - 0.1 <= s[0] <= t_to + 0.1], dtype=float)
    if len(arr) < 2:
        return None
    t = arr[:, 0]
    if len(arr) < 6 or t_to - t_from < SPINUP_S + 2 * SPEED_WIN_S:     # too short for speeds: distance only
        return {name: {'distance': float(np.interp(t_to, t, (arr[:, i] - arr[0, i]) * m_per_count[i - 1]))}
                for i, name in ((1, 'left'), (2, 'right'))}
    out = {}
    for i, name in ((1, 'left'), (2, 'right')):
        d = (arr[:, i] - arr[0, i]) * m_per_count[i - 1]
        edges = np.arange(t_from + SPINUP_S, t_to - 1e-9, SPEED_WIN_S)
        if len(edges) < 3:
            return None
        pos = np.interp(edges, t, d)
        sp = np.diff(pos) / SPEED_WIN_S
        mean = (np.interp(t_to, t, d) - np.interp(t_from + SPINUP_S, t, d)) / (t_to - t_from - SPINUP_S)
        out[name] = {
            'distance': float(np.interp(t_to, t, d)),
            'mean': float(mean),
            'min': float(sp.min()), 'max': float(sp.max()),
            'cv': float(sp.std() / abs(mean)) if abs(mean) > 1e-3 else float('inf'),
            'stopped': float(np.mean(np.abs(sp) < 0.01)),
            'series': [round(float(x), 4) for x in sp],
        }
    return out


def print_wheel_stats(ws: Optional[dict], straight: bool) -> None:
    if not ws:
        print('바퀴별: 데이터 부족 (/aircore/enc_zeroed)')
        return
    short = 'mean' not in ws['left']
    print('바퀴별 (구동 중, 짧아서 이동 거리만):' if short else
          f'바퀴별 (구동 중, 시작 {SPINUP_S:.1f} s 제외, {SPEED_WIN_S:.2f} s 구간 속도):')
    for name, label in (('left', '왼쪽'), ('right', '오른쪽')):
        w = ws[name]
        if short:
            print(f'  {label:3s}: 이동 {w["distance"]:+.3f} m')
            continue
        print(f'  {label:3s}: 이동 {w["distance"]:+.3f} m, 평균 {w["mean"]:+.3f} m/s, '
              f'구간 속도 {w["min"]:+.3f} ~ {w["max"]:+.3f} (변동 {w["cv"] * 100:.0f}%, 거의 정지한 구간 {w["stopped"] * 100:.0f}%)')
    dl, dr = ws['left']['distance'], ws['right']['distance']
    if straight and abs(dl) > 0.02:
        ratio = dr / dl
        side = '오른쪽' if ratio < 1 else '왼쪽'
        if abs(1 - ratio) < 0.02:
            print(f'  오른쪽/왼쪽 이동 비 = {ratio:.3f} → 좌우 거의 같음')
        else:
            print(f'  오른쪽/왼쪽 이동 비 = {ratio:.3f} → {side} 바퀴가 {abs(1 - ratio) * 100:.0f}% 덜 감 → {side}으로 휨')
    elif not straight and abs(dl) > 0.02:
        print(f'  |오른쪽|/|왼쪽| 이동 비 = {abs(dr) / abs(dl):.3f} (1 이 아니면 중심이 밀림)')


# ---------------- ROS node ----------------

class Calib(Node):
    def __init__(self) -> None:
        super().__init__('floor_calib')
        self.lock = threading.Lock()
        self.odom: Optional[Tuple[float, float, float, float]] = None   # x, y, cum_yaw, t
        self._prev_yaw: Optional[float] = None
        self._cum = 0.0
        self.scans: List[Tuple[float, LaserScan]] = []
        self.recording = False
        self.enc_log: List[Tuple[float, int, int]] = []
        self.odom_log: List[Tuple[float, float, float, float]] = []
        self.create_subscription(Int32MultiArray, '/aircore/enc_zeroed', self._on_enc, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            if self._prev_yaw is None:
                self._cum = yaw            # cumulative yaw starts at the absolute yaw
            else:
                self._cum += wrap(yaw - self._prev_yaw)
            self._prev_yaw = yaw
            self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, self._cum,
                         time.monotonic())
            if self.recording:
                self.odom_log.append((self.odom[3], self.odom[0], self.odom[1], self._cum))

    def _on_enc(self, msg: Int32MultiArray) -> None:
        if len(msg.data) == 2 and self.recording:
            with self.lock:
                self.enc_log.append((time.monotonic(), int(msg.data[0]), int(msg.data[1])))

    def _on_scan(self, msg: LaserScan) -> None:
        with self.lock:
            self.scans.append((time.monotonic(), msg))
            del self.scans[:-10]

    def odom_now(self):
        with self.lock:
            return self.odom

    def scan_count_and_last(self):
        with self.lock:
            return len(self.scans), (self.scans[-1] if self.scans else None)

    def wait(self, cond, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.02)
        return False

    def capture(self, laser_pose, n: int = SCANS_PER_CAPTURE, timeout: float = 3.0) -> Optional[np.ndarray]:
        """Stack the next n scans (robot must be still)."""
        t0 = time.monotonic()
        if not self.wait(lambda: sum(1 for t, _ in self.scans if t > t0) >= n, timeout):
            return None
        with self.lock:
            fresh = [m for t, m in self.scans if t > t0][:n]
        return np.vstack([scan_to_points(m, laser_pose) for m in fresh])

    def laser_pose(self, frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(BASE_FRAME, frame, Time(),
                                                 timeout=Duration(seconds=2.0))
        except Exception as e:  # noqa: BLE001 - tf2 raises several types
            self.get_logger().warn(f'TF {BASE_FRAME} <- {frame} 없음: {e}')
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def fetch_params(self, target: str, names: List[str]) -> Optional[dict]:
        cli = self.create_client(GetParameters, f'/{target}/get_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                return None
            req = GetParameters.Request()
            req.names = names
            done = threading.Event()
            fut = cli.call_async(req)
            fut.add_done_callback(lambda _f: done.set())
            if not done.wait(2.0) or fut.result() is None:
                return None
            out = {}
            for name, v in zip(names, fut.result().values):
                out[name] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value}.get(v.type)
            return out if all(out.get(n) is not None for n in names) else None
        finally:
            self.destroy_client(cli)

    def send(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        for _ in range(STOP_REPEAT):
            self.send(0.0, 0.0)
            time.sleep(0.05)


def rel_pose(start, cur) -> Tuple[float, float, float]:
    """(forward, left, yaw) of cur in the start frame; yaw is cumulative (unwrapped)."""
    x0, y0, c0, _ = start
    x, y, c, _ = cur
    yaw0 = wrap(c0)
    dx, dy = x - x0, y - y0
    return (math.cos(yaw0) * dx + math.sin(yaw0) * dy,
            -math.sin(yaw0) * dx + math.cos(yaw0) * dy, c - c0)


def ask_float(prompt: str) -> Optional[float]:
    while True:
        try:
            s = input(prompt).strip()
        except EOFError:
            return None
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            print('  숫자로 입력 (모르면 Enter)')


# ---------------- main ----------------

def parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description='2단계 바닥 보정: 정해진 직진/제자리 회전 후 /odom 과 실제 이동 비교')
    sub = ap.add_subparsers(dest='mode', required=True)
    s = sub.add_parser('straight', help='직진 → wheel_radius 보정')
    s.add_argument('--distance', type=float, default=1.0, help='odom 기준 목표 거리 [m] (기본 1.0)')
    s.add_argument('--speed', type=float, default=0.5, help='/cmd_vel linear.x [m/s] (기본 0.5)')
    r = sub.add_parser('rotate', help='제자리 회전 → wheel_separation 보정')
    r.add_argument('--angle', type=float, default=360.0, help='odom 기준 목표 회전각 [deg] (기본 360)')
    r.add_argument('--turn', type=float, default=4.6,
                   help='/cmd_vel angular.z [rad/s], 양수=좌회전 (기본 4.6, dead band 때문에 실제는 더 느림)')
    for p in (s, r):
        p.add_argument('--duty', type=int, default=None,
                       help='--speed/--turn 대신 바퀴 duty 로 지정 (실행 중 max_wheel_speed 로 환산, rotate 는 음수=우회전)')
        p.add_argument('--max-time', type=float, default=8.0, help='최대 구동 시간 [s] (기본 8)')
        p.add_argument('--yes', action='store_true', help='구동 전 확인 질문 생략')
    args = ap.parse_args(argv)
    checks = [('max_time', args.max_time)]
    if args.duty is not None:
        checks.append(('duty', abs(args.duty)))
        if args.mode == 'straight' and args.duty < 0:
            ap.error('straight 의 --duty 는 양수 (전진)')
    if args.mode == 'straight':
        checks += [('distance', args.distance), ('speed', args.speed)]
    else:
        checks += [('angle', args.angle), ('turn', abs(args.turn))]
    for key, val in checks:
        lo, hi = LIMITS[key]
        if not lo <= val <= hi:
            ap.error(f'--{key.replace("_", "-")} 는 {lo}~{hi} (입력 {val})')
    return args


def run(node: Calib, args: argparse.Namespace) -> int:
    straight = args.mode == 'straight'

    # ---- preconditions (nothing is published before these pass) ----
    print('/odom 수신 대기...')
    if not node.wait(lambda: node.odom_now() is not None, 3.0):
        print('[거부] /odom 수신 없음 → bringup 실행 중인지, ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 인지 확인')
        return 2
    if not node.wait(lambda: node.count_subscribers('/cmd_vel') >= 1, 2.0):
        print('[거부] /cmd_vel 구독자 없음 (base_controller 미실행)')
        return 2
    bc = node.fetch_params('base_controller', BC_PARAMS)
    od = node.fetch_params('odometry', OD_PARAMS)
    if bc is None or od is None:
        print('[거부] 실행 중인 base_controller/odometry 파라미터를 읽지 못함')
        return 2
    if bc['max_wheel_speed'] <= 0.0 or bc['wheel_separation'] <= 0.0:
        print('[거부] base_controller 의 max_wheel_speed/wheel_separation 이 0 → /cmd_vel 로 안 움직임')
        return 2

    if args.duty is not None:
        if abs(args.duty) > bc['max_duty']:
            print(f'[거부] --duty {args.duty} 가 실행 중 max_duty={bc["max_duty"]} 보다 큼 '
                  '(real_params.yaml 을 바꿨다면 bringup 재시작 필요)')
            return 2
        v_wheel = args.duty / 100.0 * bc['max_wheel_speed']
        if straight:
            args.speed = v_wheel
        else:
            args.turn = v_wheel / (bc['wheel_separation'] / 2.0)
    v, w = (args.speed, 0.0) if straight else (0.0, args.turn)
    duty, sent = preview_duty(v, w, bc)
    raw = max(abs(v - w * bc['wheel_separation'] / 2), abs(v + w * bc['wheel_separation'] / 2)) \
        / bc['max_wheel_speed'] * 100.0
    if raw > bc['max_duty'] + 0.5:
        print(f'[주의] 요청 duty {raw:.0f} → max_duty {bc["max_duty"]} 로 잘림')
    stop_gap = STRAIGHT_STOP_GAP + STOP_GAP_PER_SPEED * abs(v)
    if max(abs(duty[0]), abs(duty[1])) < 1:
        print('[거부] duty 가 0 → 속도를 올릴 것')
        return 2

    laser = None
    if node.wait(lambda: node.scan_count_and_last()[0] > 0, 2.0):
        _, last = node.scan_count_and_last()
        laser = node.laser_pose(last[1].header.frame_id)
    before = node.capture(laser) if laser else None
    if before is None or len(before) < 50:
        laser, before = None, None
        print('[주의] LiDAR(/scan + TF) 없음 → 장애물 정지 없음, 실제 이동은 줄자/눈으로만 측정')
    else:
        if straight:
            gap = front_gap(before)
            need = args.distance + stop_gap + 0.1
            print(f'LiDAR: 앞쪽 빈 공간 {gap:.2f} m (필요 {need:.2f} m)')
            if gap < need:
                print('[거부] 앞쪽 공간 부족 → 로봇을 옮기거나 --distance 를 줄일 것')
                return 2
        else:
            near = nearest_radius(before)
            print(f'LiDAR: 로봇 중심에서 가장 가까운 물체 {near:.2f} m (필요 {ROTATE_CLEAR_R:.2f} m 이상)')
            if near < ROTATE_CLEAR_R:
                print('[거부] 회전 반경 안에 물체 있음')
                return 2

    print(f'실행 중 설정: max_wheel_speed={bc["max_wheel_speed"]} max_duty={bc["max_duty"]} '
          f'L={bc["wheel_separation"]} r={od["wheel_radius"]} invert=({bc["invert_left"]},{bc["invert_right"]})')
    if straight:
        print(f'계획: 직진 /cmd_vel v={v:.2f} m/s → duty L={duty[0]} R={duty[1]} (전송값 M,{sent[0]},{sent[1]}), '
              f'odom {args.distance:.2f} m 에서 정지, 최대 {args.max_time:.1f} s')
    else:
        print(f'계획: 제자리 {"좌" if w > 0 else "우"}회전 /cmd_vel w={w:.2f} rad/s → duty L={duty[0]} R={duty[1]} '
              f'(전송값 M,{sent[0]},{sent[1]}), odom {args.angle:.0f}° 에서 정지, 최대 {args.max_time:.1f} s')
    stall = f'{STALL_MIN_DIST * 100:.0f} cm' if straight else f'{math.degrees(STALL_MIN_ANGLE):.0f}°'
    guard = (f'앞 {stop_gap:.2f} m 안 장애물' if straight else f'반경 {ROTATE_CLEAR_R:.2f} m 안 물체') \
        if laser else 'LiDAR 없음'
    turn = f' / {math.degrees(STRAIGHT_MAX_YAW):.0f}° 넘게 틀어짐' if straight else ''
    print(f'자동 정지: 어느 {STALL_S:.1f} s 동안이든 {stall} 못 움직임{turn} / {guard} / /odom 끊김 / Ctrl+C')
    if not args.yes:
        try:
            ok = input('로봇이 바닥에 있고 주변이 비어 있으면 yes 입력: ').strip().lower() == 'yes'
        except EOFError:
            ok = False
        if not ok:
            print('취소 (/cmd_vel 보내지 않음)')
            return 1

    # ---- drive ----
    start = node.odom_now()
    target = args.distance if straight else math.radians(args.angle)
    sign = 1.0 if (straight or w > 0) else -1.0
    with node.lock:
        node.enc_log.clear()
        node.odom_log.clear()
    node.recording = True
    time.sleep(0.2)                         # a few samples at rest
    t0 = time.monotonic()
    nxt = t0
    reason = ''
    hist = collections.deque()              # (elapsed, progress) for the rolling stall check
    try:
        while True:
            cur = node.odom_now()
            el = time.monotonic() - t0
            if time.monotonic() - cur[3] > ODOM_STALE_S:
                reason = '[중단] /odom 끊김'
                break
            fwd, _, yaw = rel_pose(start, cur)
            prog = fwd if straight else sign * yaw
            if prog >= target:
                reason = '목표 도달 (odom 기준)'
                break
            if el > args.max_time:
                reason = f'[중단] 시간 초과 {args.max_time:.1f} s (odom {prog if straight else math.degrees(prog):.2f})'
                break
            moved = f'{prog * 100:+.1f} cm' if straight else f'{math.degrees(prog):+.1f}°'
            if prog < -(STALL_MIN_DIST if straight else STALL_MIN_ANGLE):
                reason = f'[중단] 명령과 반대 방향으로 움직임 (odom {moved}) → invert 설정 확인 필요'
                break
            hist.append((el, prog))
            while len(hist) > 1 and hist[1][0] <= el - STALL_S:
                hist.popleft()
            min_gain = STALL_MIN_DIST if straight else STALL_MIN_ANGLE
            if el > STALL_S and prog - hist[0][1] < min_gain:
                if prog < min_gain:
                    reason = (f'[중단] {STALL_S:.1f} s 동안 odom {moved} 만 움직임 → duty {duty} 가 바닥에서 '
                              f'dead band 이하일 수 있음')
                else:
                    gain = prog - hist[0][1]
                    gained = f'{gain * 100:+.1f} cm' if straight else f'{math.degrees(gain):+.1f}°'
                    reason = (f'[중단] 움직이다 멈춤: 최근 {STALL_S:.1f} s 동안 odom {gained} (누적 {moved}) '
                              f'→ 명령은 그대로인데 바퀴가 섬 (드라이버/전원 차단 또는 걸림 의심)')
                break
            if straight and abs(yaw) > STRAIGHT_MAX_YAW:
                side = '왼쪽으로 (오른쪽 바퀴가 더 감)' if yaw > 0 else '오른쪽으로 (왼쪽 바퀴가 더 감)'
                reason = f'[중단] 직진 중 {math.degrees(yaw):+.0f}° 틀어짐 → {side}'
                break
            if laser:
                _, last = node.scan_count_and_last()
                if time.monotonic() - last[0] > SCAN_STALE_S:
                    reason = '[중단] /scan 끊김'
                    break
                pts = scan_to_points(last[1], laser)
                if straight and front_gap(pts) < stop_gap:
                    reason = f'[중단] 앞 {front_gap(pts):.2f} m 장애물'
                    break
                if not straight and nearest_radius(pts) < ROTATE_CLEAR_R:
                    reason = f'[중단] 반경 {nearest_radius(pts):.2f} m 물체'
                    break
            node.send(v, w)
            nxt += 1.0 / SEND_HZ
            time.sleep(max(0.0, nxt - time.monotonic()))
    finally:
        drive_s = time.monotonic() - t0     # before the 0.5 s of stop messages
        node.stop()
    print(f'정지: {reason} (구동 {drive_s:.2f} s)')
    at_stop = rel_pose(start, node.odom_now())
    t_end = t0 + drive_s
    time.sleep(SETTLE_S)
    fwd, left, yaw = rel_pose(start, node.odom_now())
    node.recording = False
    after = node.capture(laser) if laser else None
    m_per_count = (2 * math.pi * od['wheel_radius'] / od['ticks_per_rev_left'],
                   2 * math.pi * od['wheel_radius'] / od['ticks_per_rev_right'])
    with node.lock:
        enc_log = list(node.enc_log)
        odom_log = list(node.odom_log)
    ws = wheel_stats(enc_log, t0, t_end, m_per_count)
    summary = {'mode': args.mode, 'duty': list(duty), 'sent': list(sent), 'v': v, 'w': w,
               'reason': reason, 'drive_s': drive_s, 'params': {**bc, **od},
               'odom': {'fwd': fwd, 'left': left, 'yaw_deg': math.degrees(yaw)}, 'wheels': ws}

    # ---- results ----
    print('\n================ 결과 ================')
    if straight:
        disp = math.hypot(fwd, left)
        print(f'odom: 앞 {fwd:+.3f} m, 옆 {left:+.3f} m → 이동 {disp:.3f} m '
              f'(정지 명령 후 관성 {fwd - at_stop[0]:+.3f} m), 방향 변화 {math.degrees(yaw):+.1f}°')
        print(f'      평균 속도 {at_stop[0] / drive_s:.3f} m/s (명령 {v:.2f})')
        print_wheel_stats(ws, True)
        lidar_d = None
        if after is not None:
            res = icp(after, before, yaw, np.array([fwd, left]))
            if res:
                th, t, inl, rms = res
                lidar_d = float(np.hypot(*t))
                good = inl > 0.5 and rms < 0.03
                summary['lidar'] = {'fwd': float(t[0]), 'left': float(t[1]), 'yaw_deg': math.degrees(th),
                                    'inlier': inl, 'rms': rms}
                print(f'LiDAR: 앞 {t[0]:+.3f} m, 옆 {t[1]:+.3f} m → 이동 {lidar_d:.3f} m, 방향 변화 '
                      f'{math.degrees(th):+.1f}°  (일치 {inl * 100:.0f}%, rms {rms * 1000:.0f} mm'
                      f'{"" if good else ", 신뢰 낮음 → 줄자 값 사용 권장"})')
                if not good:
                    lidar_d = None
            else:
                print('LiDAR: 스캔 정합 실패')
        tape = ask_float('줄자로 잰 실제 이동 거리 [m] (모르면 Enter' + (', LiDAR 값 사용' if lidar_d else '') + '): ')
        summary['tape'] = tape
        actual = tape if tape is not None else lidar_d
        if actual and disp > 0.05:
            ratio = actual / disp
            r_new = od['wheel_radius'] * ratio
            src = '줄자' if tape is not None else 'LiDAR'
            print(f'\n실제/odom = {actual:.3f}/{disp:.3f} = {ratio:.4f} ({src} 기준)')
            print(f'wheel_radius 제안: {od["wheel_radius"]:.4f} → {r_new:.4f} m')
            if abs(ratio - 1.0) > 0.15:
                print('[주의] 15% 이상 차이 → 측정 실수 가능성, 한 번 더 측정 권장')
            if lidar_d and tape is not None:
                print(f'(참고: 줄자 {tape:.3f} vs LiDAR {lidar_d:.3f} m)')
            print(f'적용: ros2 run aircore_base apply_measurements.py --wheel-radius {r_new:.4f}  → bringup 재시작')
        if abs(math.degrees(yaw)) > 3.0:
            print(f'[참고] 직진 중 {math.degrees(yaw):+.1f}° 틀어짐 (좌우 바퀴 속도 차이, base_controller 는 개루프)')
    else:
        print(f'odom: 회전 {math.degrees(yaw):+.1f}° (정지 명령 후 관성 {math.degrees(yaw - at_stop[2]):+.1f}°), '
              f'중심 이동 {math.hypot(fwd, left) * 100:.1f} cm')
        print(f'      평균 각속도 {math.degrees(at_stop[2]) / drive_s:+.0f}°/s (명령 {math.degrees(w):+.0f}°/s)')
        print_wheel_stats(ws, False)
        lidar_total = None
        if after is not None:
            th0 = coarse_rotation(after, before)
            res = icp(after, before, th0, np.zeros(2))
            if res:
                th, t, inl, rms = res
                good = inl > 0.5 and rms < 0.03
                lidar_total = resolve_turns(th, yaw)
                summary['lidar'] = {'yaw_deg': math.degrees(lidar_total), 'shift': float(np.hypot(*t)),
                                    'inlier': inl, 'rms': rms}
                print(f'LiDAR: 회전 {math.degrees(lidar_total):+.1f}° (360° 나머지 {math.degrees(th):+.1f}°), '
                      f'중심 이동 {np.hypot(*t) * 100:.1f} cm  (일치 {inl * 100:.0f}%, rms {rms * 1000:.0f} mm'
                      f'{"" if good else ", 신뢰 낮음 → 눈으로 잰 값 사용 권장"})')
                if not good:
                    lidar_total = None
            else:
                print('LiDAR: 스캔 정합 실패')
        eye = ask_float('실제 회전각 [deg] (부호 포함, 모르면 Enter' + (', LiDAR 값 사용' if lidar_total else '') + '): ')
        summary['eye_deg'] = eye
        actual = math.radians(eye) if eye is not None else lidar_total
        if actual and abs(actual) > math.radians(10):
            ratio = yaw / actual
            L_new = od['wheel_separation'] * ratio
            src = '눈' if eye is not None else 'LiDAR'
            print(f'\nodom/실제 = {math.degrees(yaw):.1f}/{math.degrees(actual):.1f} = {ratio:.4f} ({src} 기준)')
            print(f'wheel_separation 제안: {od["wheel_separation"]:.4f} → {L_new:.4f} m')
            if abs(ratio - 1.0) > 0.25:
                print('[주의] 25% 이상 차이 → 측정 실수 가능성, 한 번 더 측정 권장')
            print(f'적용: ros2 run aircore_base apply_measurements.py --wheel-separation {L_new:.4f}  → bringup 재시작')
    save_log(summary, enc_log, odom_log, t0)
    return 0 if reason.startswith('목표') else 3


def save_log(summary: dict, enc_log, odom_log, t0: float) -> None:
    """~/aircore_ws/calib_logs/<time>_<mode>_duty<N>.json (summary + raw samples, t relative to start)."""
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        name = f'{time.strftime("%Y%m%d-%H%M%S")}_{summary["mode"]}_duty{max(abs(d) for d in summary["duty"])}.json'
        summary['enc'] = [(round(t - t0, 4), l, r) for t, l, r in enc_log]
        summary['odom_samples'] = [(round(t - t0, 4), round(x, 5), round(y, 5), round(c, 5)) for t, x, y, c in odom_log]
        with open(os.path.join(LOG_DIR, name), 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, default=float)
        print(f'\n기록 저장: {os.path.join(LOG_DIR, name)}')
    except OSError as e:
        print(f'[주의] 기록 저장 실패: {e}')


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv if argv is None else argv
    args = parse_args(remove_ros_args(argv)[1:])
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = Calib()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_spin = threading.Event()

    def spin() -> None:
        try:
            while not stop_spin.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.05)
        except ExternalShutdownException:
            pass

    def _raise(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _raise)
    signal.signal(signal.SIGHUP, _raise)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    code = 0
    try:
        code = run(node, args)
    except KeyboardInterrupt:
        print('\n중단됨 → 정지 명령 전송')
        node.stop()
        code = 130
    finally:
        node.stop()
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
