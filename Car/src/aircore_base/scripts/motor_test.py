#!/usr/bin/env python3
"""Short, bounded motor test on the real STM32 (wheels lifted): motor direction + max_wheel_speed.

Talks to the STM32 directly with the base_controller protocol, so base_controller must NOT be
running (checked). Sends "M,left,right" at 20 Hz for at most 3 s per step, then "M,0,0".
Safety: |duty| <= max_duty from the params file, the drive stops if encoder lines stop arriving,
Ctrl+C / SIGTERM / errors always end with "M,0,0" x5, and the STM32 itself stops the motors
when UART is silent for 300 ms.

Direction follows base_controller: the commanded duty is negated when invert_left/right is true,
so "forward" here means what base_controller would send for a forward /cmd_vel.

  motor_test.py                               # duty 0: sends only M,0,0 (serial/encoder check)
  motor_test.py --left 15                     # left wheel only, 2 s
  motor_test.py --right 15 --duration 1.5     # right wheel only
  motor_test.py --sweep 15,20,25,30           # both wheels, one step per duty -> max_wheel_speed
"""
import argparse
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import serial
import yaml

SEND_HZ = 20.0
MAX_DURATION_S = 3.0
MIN_DURATION_S = 0.5
SETTLE_S = 0.6              # skip spin-up before measuring steady speed
COAST_S = 0.8               # keep sending M,0,0 and watch encoders after each step
PAUSE_S = 1.0               # stopped time between sweep steps
STOP_REPEAT = 5
ENC_STALE_S = 0.5           # abort the drive if no ENC line for this long
MOVING_REV = 0.05           # < 5% of a revolution counts as "did not move"
MAX_SWEEP_STEPS = 6


# ---------------- helpers (same conversions as base_controller / odometry) ----------------

def to_int32(value: int) -> int:
    if not -2**31 <= value < 2**32:
        raise ValueError(f'out of int32/uint32 range: {value}')
    return value - 2**32 if value >= 2**31 else value


def wrap_int32_diff(cur: int, prev: int) -> int:
    dn = cur - prev
    if dn >= 2**31:
        dn -= 2**32
    elif dn < -2**31:
        dn += 2**32
    return dn


def default_params_file() -> Path:
    return Path(__file__).resolve().parent.parent / 'config' / 'real_params.yaml'


def load_params(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    bc = (data.get('base_controller') or {}).get('ros__parameters') or {}
    od = (data.get('odometry') or {}).get('ros__parameters') or {}
    lidar = (data.get('sllidar_node') or {}).get('ros__parameters') or {}
    return {
        'port': bc.get('serial_port'),
        'baud': int(bc.get('baudrate', 115200)),
        'max_duty': max(0, min(100, int(bc.get('max_duty', 30)))),
        'invert_left': bool(bc.get('invert_left', False)),
        'invert_right': bool(bc.get('invert_right', False)),
        'invert_enc_left': bool(od.get('invert_enc_left', False)),
        'invert_enc_right': bool(od.get('invert_enc_right', False)),
        'radius': float(od.get('wheel_radius', 0.0)),
        'ticks_left': int(od.get('ticks_per_rev_left', 0)),
        'ticks_right': int(od.get('ticks_per_rev_right', 0)),
        'lidar_port': lidar.get('serial_port'),
    }


def port_holders(port: str) -> List[str]:
    """Other processes that have the serial device open, or a running base_controller."""
    real = os.path.realpath(port)
    hits = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        holds = False
        try:
            for fd in os.listdir(f'/proc/{pid}/fd'):
                try:
                    if os.readlink(f'/proc/{pid}/fd/{fd}') == real:
                        holds = True
                        break
                except OSError:
                    pass
        except OSError:
            pass
        if holds or 'aircore_base/base_controller' in cmd:
            hits.append(f'pid {pid}: {cmd.strip()[:120]}')
    return hits


# ---------------- encoder reader ----------------

class EncReader(threading.Thread):
    """Reads ENC lines; keeps forward-positive cumulative counts since start (invert_enc applied)."""

    def __init__(self, ser: serial.Serial, inv_l: bool, inv_r: bool) -> None:
        super().__init__(daemon=True)
        self.ser = ser
        self.inv = (inv_l, inv_r)
        self.lock = threading.Lock()
        self.samples: List[Tuple[float, int, int]] = []
        self.running = True
        self.error: Optional[Exception] = None
        self.bad_lines = 0
        self._prev: Optional[Tuple[int, int]] = None
        self._acc = [0, 0]

    def run(self) -> None:
        buf = b''
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError) as e:
                self.error = e
                return
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._handle(raw)
            if len(buf) > 256:
                buf = b''

    def _handle(self, raw: bytes) -> None:
        line = raw.decode('ascii', errors='replace').strip().strip('\x00')
        if not line:
            return
        parts = line.split(',')
        try:
            if len(parts) != 3 or parts[0] != 'ENC':
                raise ValueError
            left, right = to_int32(int(parts[1])), to_int32(int(parts[2]))
        except ValueError:
            self.bad_lines += 1
            return
        now = time.monotonic()
        if self._prev is not None:
            dl = wrap_int32_diff(left, self._prev[0])
            dr = wrap_int32_diff(right, self._prev[1])
            self._acc[0] += -dl if self.inv[0] else dl
            self._acc[1] += -dr if self.inv[1] else dr
        self._prev = (left, right)
        with self.lock:
            self.samples.append((now, self._acc[0], self._acc[1]))

    def snapshot(self) -> List[Tuple[float, int, int]]:
        with self.lock:
            return list(self.samples)

    def last_age(self) -> float:
        with self.lock:
            return time.monotonic() - self.samples[-1][0] if self.samples else float('inf')


def value_at(samples, t: float, idx: int) -> Optional[int]:
    """Cumulative count of the last sample at or before t."""
    best = None
    for s in samples:
        if s[0] <= t:
            best = s[idx]
        else:
            break
    return best


def rate(samples, t0: float, t1: float, idx: int) -> Optional[float]:
    pts = [s for s in samples if t0 <= s[0] <= t1]
    if len(pts) < 2 or pts[-1][0] <= pts[0][0]:
        return None
    return (pts[-1][idx] - pts[0][idx]) / (pts[-1][0] - pts[0][0])


# ---------------- driving ----------------

class Driver:
    def __init__(self, ser: serial.Serial, reader: EncReader, p: dict) -> None:
        self.ser, self.reader, self.p = ser, reader, p

    def _write(self, left: int, right: int) -> None:
        self.ser.write(f'M,{left},{right}\n'.encode('ascii'))

    def stop(self) -> None:
        for _ in range(STOP_REPEAT):
            try:
                self._write(0, 0)
            except (serial.SerialException, OSError):
                pass
            time.sleep(0.02)

    def hold(self, left: int, right: int, seconds: float, watch_enc: bool) -> None:
        period = 1.0 / SEND_HZ
        t0 = time.monotonic()
        nxt = t0
        while time.monotonic() - t0 < seconds:
            self._write(left, right)
            if watch_enc and self.reader.last_age() > ENC_STALE_S:
                raise RuntimeError(f'ENC 수신이 {ENC_STALE_S}s 이상 끊김 → 즉시 정지')
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def step(self, duty_l: int, duty_r: int, duration: float) -> dict:
        cmd_l = -duty_l if self.p['invert_left'] else duty_l
        cmd_r = -duty_r if self.p['invert_right'] else duty_r
        t_start = time.monotonic()
        try:
            self.hold(cmd_l, cmd_r, duration, watch_enc=True)
        finally:
            t_end = time.monotonic()
            self.stop()
        self.hold(0, 0, COAST_S, watch_enc=False)
        t_coast = time.monotonic()

        s = self.reader.snapshot()
        res = {'duty': (duty_l, duty_r), 'cmd': (cmd_l, cmd_r), 'duration': t_end - t_start}
        for idx, name in ((1, 'left'), (2, 'right')):
            c0, c1 = value_at(s, t_start, idx), value_at(s, t_end, idx)
            c2 = value_at(s, t_coast, idx)
            res[name] = {
                'counts': None if c0 is None or c1 is None else c1 - c0,
                'coast': None if c1 is None or c2 is None else c2 - c1,
                'steady_cps': rate(s, t_start + SETTLE_S, t_end, idx),
                'after_cps': rate(s, t_coast - 0.3, t_coast, idx),
            }
        return res


def describe(res: dict, p: dict) -> List[str]:
    """Print one step and return recommendation lines."""
    notes = []
    circ = 2.0 * math.pi * p['radius']
    print(f'  명령 duty L={res["duty"][0]} R={res["duty"][1]}  (전송값 M,{res["cmd"][0]},{res["cmd"][1]}, '
          f'{res["duration"]:.2f}s)')
    for name, label, ticks, inv_key, i in (('left', '왼쪽', p['ticks_left'], 'invert_left', 0),
                                           ('right', '오른쪽', p['ticks_right'], 'invert_right', 1)):
        r = res[name]
        duty = res['duty'][i]
        if r['counts'] is None:
            print(f'  {label}: 엔코더 데이터 부족')
            continue
        rev = r['counts'] / ticks if ticks > 0 else float('nan')
        speed = ''
        if r['steady_cps'] is not None and ticks > 0 and p['radius'] > 0:
            rps = r['steady_cps'] / ticks
            speed = f', 정상상태 {rps:+.2f} rev/s = {rps * circ:+.3f} m/s'
            r['speed'] = rps * circ
        moved = ticks > 0 and abs(rev) >= MOVING_REV
        if duty == 0:
            verdict = '명령 0 → 정지 유지' if not moved else '[경고] 명령하지 않았는데 돎 → 좌우 모터 배선이 바뀌었을 수 있음'
        elif not moved:
            verdict = '[경고] 회전 없음 → duty 가 정지마찰보다 낮거나 모터 전원/배선 문제'
        elif (rev > 0) == (duty > 0):
            verdict = '방향 정상 (엔코더 기준 전진)' if duty > 0 else '방향 정상 (엔코더 기준 후진)'
        else:
            verdict = f'[방향 반대] → {inv_key} 를 {p[inv_key]} 에서 {not p[inv_key]} 로 바꿔야 함'
            notes.append(f'{inv_key}: {p[inv_key]} → {not p[inv_key]}')
        coast = f', 정지 후 관성 {r["coast"]:+d}' if r['coast'] is not None else ''
        after = r['after_cps']
        stopped = '' if after is None else (' (정지 확인)' if abs(after) < 5 else f' [경고] 정지 후에도 {after:+.0f} counts/s')
        print(f'  {label}: {r["counts"]:+d} counts = {rev:+.2f} rev{speed}{coast}{stopped}')
        print(f'         {verdict}')
    return notes


def fit_max_speed(points: List[Tuple[int, float]]) -> Optional[float]:
    """Least squares through the origin: v = k*duty (base_controller's model) -> v at duty 100."""
    num = sum(d * v for d, v in points)
    den = sum(d * d for d, _ in points)
    return None if den == 0 else num / den * 100.0


# ---------------- main ----------------

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description='바퀴를 띄운 상태에서 짧고 제한된 모터 테스트 (방향 확인 + max_wheel_speed 측정). '
                    'base_controller 가 실행 중이면 거부한다. 인자 없이 실행하면 M,0,0 만 보낸다.')
    ap.add_argument('--left', type=int, default=0, help='왼쪽 duty (기본 0)')
    ap.add_argument('--right', type=int, default=0, help='오른쪽 duty (기본 0)')
    ap.add_argument('--duration', type=float, default=2.0,
                    help=f'한 단계 구동 시간 [s] ({MIN_DURATION_S}~{MAX_DURATION_S}, 기본 2.0)')
    ap.add_argument('--sweep', type=str, default=None,
                    help='두 바퀴 같은 duty 로 차례로 구동, 예: 15,20,25,30 → max_wheel_speed 추정')
    ap.add_argument('--yes', action='store_true', help='구동 전 확인 질문 생략')
    ap.add_argument('--params', type=Path, default=None,
                    help=f'파라미터 yaml (기본: {default_params_file()})')
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    params_path = (args.params or default_params_file()).expanduser().resolve()
    p = load_params(params_path)
    port = p['port']

    # ---- static safety checks (nothing is opened before these pass) ----
    errors = []
    if not port:
        errors.append('base_controller.serial_port 가 없음')
    else:
        if p['lidar_port'] and os.path.realpath(port) == os.path.realpath(p['lidar_port']):
            errors.append(f'serial_port 가 LiDAR 포트와 같음: {port}')
        if 'CP2102' in port:
            errors.append(f'serial_port 가 CP2102N(LiDAR) 경로임: {port}')
        if not os.path.exists(port):
            errors.append(f'포트 없음: {port}')
    if not MIN_DURATION_S <= args.duration <= MAX_DURATION_S:
        errors.append(f'--duration 은 {MIN_DURATION_S}~{MAX_DURATION_S}s')
    steps: List[Tuple[int, int]] = []
    if args.sweep:
        if args.left or args.right:
            errors.append('--sweep 과 --left/--right 는 함께 쓸 수 없음')
        try:
            duties = [int(x) for x in args.sweep.split(',') if x.strip()]
        except ValueError:
            duties = []
            errors.append(f'--sweep 형식 오류: {args.sweep}')
        if not 1 <= len(duties) <= MAX_SWEEP_STEPS:
            errors.append(f'--sweep 단계는 1~{MAX_SWEEP_STEPS}개')
        if any(d <= 0 for d in duties):
            errors.append('--sweep duty 는 양수 (전진)')
        steps = [(d, d) for d in duties]
    else:
        steps = [(args.left, args.right)]
    for dl, dr in steps:
        if abs(dl) > p['max_duty'] or abs(dr) > p['max_duty']:
            errors.append(f'duty {dl},{dr} 가 max_duty={p["max_duty"]} 초과')
    holders = port_holders(port) if port and os.path.exists(port) else []
    if holders:
        errors.append('다른 프로세스가 STM32 포트를 사용 중 (base_controller/launch 를 먼저 종료):\n    '
                      + '\n    '.join(holders))
    if errors:
        for e in errors:
            print(f'[거부] {e}', file=sys.stderr)
        return 2

    moving = any(dl or dr for dl, dr in steps)
    print(f'port={port}  max_duty={p["max_duty"]}  invert(motor)=({p["invert_left"]},{p["invert_right"]})  '
          f'invert_enc=({p["invert_enc_left"]},{p["invert_enc_right"]})  '
          f'ticks=({p["ticks_left"]},{p["ticks_right"]})  r={p["radius"]}')
    print('계획: ' + ', '.join(f'[L={dl} R={dr} {args.duration:.1f}s]' for dl, dr in steps))
    if moving and not args.yes:
        if input('바퀴가 바닥에서 떠 있고 주변에 걸릴 것이 없으면 yes 입력: ').strip().lower() != 'yes':
            print('취소 (아무것도 보내지 않음)')
            return 1

    def _raise(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _raise)
    signal.signal(signal.SIGHUP, _raise)

    ser = serial.Serial(port, p['baud'], bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, timeout=0.1, write_timeout=0.1,
                        xonxoff=False, rtscts=False, dsrdtr=False)
    ser.reset_input_buffer()
    reader = EncReader(ser, p['invert_enc_left'], p['invert_enc_right'])
    reader.start()
    drv = Driver(ser, reader, p)
    exit_code = 0
    results = []
    try:
        drv.stop()  # stop first, like base_controller
        drv.hold(0, 0, 1.0, watch_enc=False)
        if len(reader.snapshot()) < 3:
            print(f'[중단] 1s 동안 ENC 수신 {len(reader.snapshot())}개 → 구동하지 않음 '
                  '(STM32 전원/펌웨어/배선 확인)', file=sys.stderr)
            return 3
        print(f'ENC 수신 확인 ({len(reader.snapshot())}개/1s)')
        for i, (dl, dr) in enumerate(steps):
            if i:
                drv.hold(0, 0, PAUSE_S, watch_enc=False)
            print(f'\n--- 단계 {i + 1}/{len(steps)} ---')
            res = drv.step(dl, dr, args.duration)
            notes = describe(res, p)
            results.append((res, notes))
    except KeyboardInterrupt:
        print('\n중단됨 → 정지 명령 전송')
        exit_code = 130
    except (RuntimeError, serial.SerialException, OSError) as e:
        print(f'\n[중단] {e}', file=sys.stderr)
        exit_code = 4
    finally:
        drv.stop()
        reader.running = False
        reader.join(timeout=1.0)
        try:
            ser.close()
        except Exception:
            pass
        print('M,0,0 x5 전송 후 포트 닫음')

    if args.sweep and results:
        print('\n================ sweep 결과 (바퀴 띄운 무부하 상태) ================')
        print(' duty   왼쪽 m/s   오른쪽 m/s')
        pts_l, pts_r = [], []
        for res, _ in results:
            vl = res['left'].get('speed')
            vr = res['right'].get('speed')
            d = res['duty'][0]
            print(f' {d:4d}   {"-" if vl is None else f"{vl:+.3f}":>8}   {"-" if vr is None else f"{vr:+.3f}":>8}')
            if vl is not None and vl > 0:
                pts_l.append((d, vl))
            if vr is not None and vr > 0:
                pts_r.append((d, vr))
        ml, mr = fit_max_speed(pts_l), fit_max_speed(pts_r)
        if ml and mr:
            avg = (ml + mr) / 2
            print(f'max_wheel_speed (duty 100 환산, 원점 통과 직선 근사): 왼쪽 {ml:.3f}  오른쪽 {mr:.3f}  '
                  f'평균 {avg:.3f} m/s  (좌우 차이 {abs(ml - mr) / avg * 100:.1f}%)')
            print('주의: 무부하 값이라 바닥 주행 시 실제 속도는 더 낮음 → 2단계에서 재보정')
        else:
            print('전진 속도가 측정된 단계가 부족해 max_wheel_speed 를 계산하지 못함')
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
