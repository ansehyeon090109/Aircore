#!/usr/bin/env python3
"""Write measured values into config/real_params.yaml, keeping both sections consistent.

  --wheel-radius      -> base_controller.wheel_radius     + odometry.wheel_radius
  --wheel-separation  -> base_controller.wheel_separation + odometry.wheel_separation
  --ticks-left        -> odometry.ticks_per_rev_left
  --ticks-right       -> odometry.ticks_per_rev_right

Keys without an argument are left untouched. Edits are line-based so comments and
layout survive. Zero, negative, non-finite or out-of-range values are rejected.
"""
import argparse
import datetime
import math
import os
import re
import sys
import tempfile
from pathlib import Path

import yaml

# Plausibility limits (catch unit mistakes such as mm/cm instead of m).
LIMITS = {
    'wheel_radius': (0.005, 0.5),       # [m]
    'wheel_separation': (0.05, 2.0),    # [m]
    'ticks': (1, 1_000_000),            # counts per wheel revolution
}

# Keys whose value must be > 0 for /odom; used for the status line in the header.
ODOM_KEYS = ('wheel_radius', 'wheel_separation', 'ticks_per_rev_left', 'ticks_per_rev_right')
STATUS_PREFIX = '# 상태:'

SECTION_RE = re.compile(r'^(?P<name>[A-Za-z_]\w*):\s*(#.*)?$')
KEY_RE = re.compile(r'^(?P<head>(?P<indent>\s+)(?P<key>[A-Za-z_]\w*):)(?P<rest>.*)$')
# Source tag in a value's comment; replaced by "실측 <date>" when the value is (re)applied.
MEASURED_TAG_RE = re.compile(r'미측정|실측 \d{4}-\d{2}-\d{2}|CAD값\([^)]*\)')


def default_params_file() -> Path:
    # Resolve symlinks (colcon --symlink-install) so the source file under src/ is edited.
    return Path(__file__).resolve().parent.parent / 'config' / 'real_params.yaml'


def check_float(name: str, value: float, unit: str) -> float:
    lo, hi = LIMITS[name]
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name}={value}: 0 이하/비정상 값은 거부')
    if not lo <= value <= hi:
        raise ValueError(f'{name}={value} {unit}: 허용 범위 {lo}~{hi} {unit} 밖 '
                         f'(단위가 m 인지 확인, 예: 3 cm → 0.03)')
    return float(value)


def check_ticks(name: str, value: int) -> int:
    lo, hi = LIMITS['ticks']
    if value <= 0:
        raise ValueError(f'{name}={value}: 0 이하 값은 거부')
    if not lo <= value <= hi:
        raise ValueError(f'{name}={value}: 허용 범위 {lo}~{hi} 밖')
    return int(value)


def build_updates(args: argparse.Namespace) -> dict:
    """{(section, key): python value}"""
    updates = {}
    if args.wheel_radius is not None:
        v = check_float('wheel_radius', args.wheel_radius, 'm')
        updates[('base_controller', 'wheel_radius')] = v
        updates[('odometry', 'wheel_radius')] = v
    if args.wheel_separation is not None:
        v = check_float('wheel_separation', args.wheel_separation, 'm')
        updates[('base_controller', 'wheel_separation')] = v
        updates[('odometry', 'wheel_separation')] = v
    if args.ticks_left is not None:
        updates[('odometry', 'ticks_per_rev_left')] = check_ticks('ticks_left', args.ticks_left)
    if args.ticks_right is not None:
        updates[('odometry', 'ticks_per_rev_right')] = check_ticks('ticks_right', args.ticks_right)
    return updates


def format_value(v) -> str:
    # floats always keep a decimal point (ROS parameter type must stay double, ticks stay int)
    return repr(float(v)) if isinstance(v, float) else str(int(v))


def rewrite_line(line: str, m: re.Match, value, today: str) -> str:
    rest = m.group('rest')
    comment = ''
    comment_col = None
    hash_pos = rest.find('#')
    if hash_pos >= 0:
        comment = MEASURED_TAG_RE.sub(f'실측 {today}', rest[hash_pos:])
        comment_col = len(m.group('head')) + hash_pos
    new = f'{m.group("head")} {format_value(value)}'
    if comment:
        pad = max(comment_col - len(new), 1) if comment_col is not None else 1
        new = new + ' ' * pad + comment
    return new


def apply(text: str, updates: dict, today: str) -> tuple:
    lines = text.split('\n')
    section = None
    applied = {}
    for i, line in enumerate(lines):
        sm = SECTION_RE.match(line)
        if sm:
            section = sm.group('name')
            continue
        km = KEY_RE.match(line)
        if not km or section is None:
            continue
        target = (section, km.group('key'))
        if target in updates:
            if target in applied:
                raise ValueError(f'{section}.{km.group("key")} 키가 파일에 두 번 있음')
            old = km.group('rest').split('#', 1)[0].strip()
            lines[i] = rewrite_line(line, km, updates[target], today)
            applied[target] = old
    missing = [f'{s}.{k}' for (s, k) in updates if (s, k) not in applied]
    if missing:
        raise ValueError(f'파일에서 키를 찾지 못함: {", ".join(missing)}')
    return '\n'.join(lines), applied


def update_status_line(text: str, data: dict) -> str:
    odom = ((data.get('odometry') or {}).get('ros__parameters') or {})
    zero = [k for k in ODOM_KEYS if not (isinstance(odom.get(k), (int, float)) and odom.get(k) > 0)]
    if zero:
        status = (f'{STATUS_PREFIX} 미측정 - 값 채우기 전에는 /odom 이 나오지 않음 '
                  f'(0 인 키: {", ".join(zero)})')
    else:
        status = f'{STATUS_PREFIX} 실측값 입력 완료 (apply_measurements.py 로 갱신)'
    lines = text.split('\n')
    for i, line in enumerate(lines):
        if line.startswith(STATUS_PREFIX):
            lines[i] = status
            break
    return '\n'.join(lines)


def verify(data: dict, updates: dict) -> None:
    for (section, key), expected in updates.items():
        got = ((data.get(section) or {}).get('ros__parameters') or {}).get(key)
        if got != expected or type(got) is not type(expected):
            raise ValueError(f'검증 실패: {section}.{key} = {got!r} (기대값 {expected!r})')
    bc = (data.get('base_controller') or {}).get('ros__parameters') or {}
    od = (data.get('odometry') or {}).get('ros__parameters') or {}
    for key in ('wheel_radius', 'wheel_separation'):
        if bc.get(key) != od.get(key):
            print(f'[경고] {key} 가 두 섹션에서 다름: base_controller={bc.get(key)} '
                  f'odometry={od.get(key)} (이 키도 인자로 넘겨서 맞추세요)')


def write_atomic(path: Path, text: str) -> None:
    mode = path.stat().st_mode & 0o777
    fd, tmp = tempfile.mkstemp(prefix='.real_params.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
        os.chmod(tmp, mode)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(
        description='실측값을 real_params.yaml 의 base_controller / odometry 섹션에 일관되게 기록한다. '
                    '인자로 넘기지 않은 키는 건드리지 않는다.')
    parser.add_argument('--wheel-radius', type=float, help='[m] 바퀴 반지름 (두 섹션 모두 갱신)')
    parser.add_argument('--wheel-separation', type=float,
                        help='[m] 좌우 바퀴 접지점 중심 간 거리 (두 섹션 모두 갱신)')
    parser.add_argument('--ticks-left', type=int, help='왼쪽 바퀴 1회전당 카운트 (odometry)')
    parser.add_argument('--ticks-right', type=int, help='오른쪽 바퀴 1회전당 카운트 (odometry)')
    parser.add_argument('--file', type=Path, default=None,
                        help=f'대상 yaml (기본: {default_params_file()})')
    args = parser.parse_args()

    if all(v is None for v in (args.wheel_radius, args.wheel_separation,
                               args.ticks_left, args.ticks_right)):
        parser.error('갱신할 값을 하나 이상 넘겨야 함')

    path = (args.file or default_params_file()).expanduser().resolve()
    if not path.is_file():
        print(f'[오류] 파일 없음: {path} (--file 로 지정)', file=sys.stderr)
        return 1

    try:
        updates = build_updates(args)
        text = path.read_text(encoding='utf-8')
        today = datetime.date.today().isoformat()
        new_text, applied = apply(text, updates, today)
        new_text = update_status_line(new_text, yaml.safe_load(new_text) or {})
        data = yaml.safe_load(new_text) or {}
        verify(data, updates)
    except (ValueError, yaml.YAMLError) as e:
        print(f'[거부] {e}  → 파일을 변경하지 않음', file=sys.stderr)
        return 2

    write_atomic(path, new_text)
    print(f'갱신: {path}')
    for (section, key), value in updates.items():
        print(f'  {section}.{key}: {applied[(section, key)]} -> {format_value(value)}')
    for line in new_text.split('\n'):
        if line.startswith(STATUS_PREFIX):
            print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
