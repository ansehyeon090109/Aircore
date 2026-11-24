#!/usr/bin/env python3
"""Interactive ticks_per_rev measurement using ROS topics only.

Needs base_controller + odometry running (e.g. odom_view.launch.py rviz:=false).
Uses /aircore/enc_zeroed (direction-corrected counts since last zero) and
/aircore/reset_odom. It never opens a serial port and never publishes /cmd_vel.

Per wheel: reset_odom -> roll the wheel forward by hand N turns -> Enter -> read counts.
Note: reset_odom also resets the odometry pose to 0.
"""
import argparse
import sys
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

ZEROED_TOPIC = '/aircore/enc_zeroed'
RESET_SERVICE = '/aircore/reset_odom'
FIRST_MSG_TIMEOUT_S = 5.0
SERVICE_TIMEOUT_S = 3.0
STALE_AFTER_S = 0.5           # enc_zeroed should arrive at ~20 Hz
CROSSTALK_RATIO = 0.10        # other wheel moved > 10% of the measured wheel -> warn
SETTLE_S = 0.3                # let the last sample arrive after Enter

WHEELS = (('left', 0, '왼쪽', 'invert_enc_left'), ('right', 1, '오른쪽', 'invert_enc_right'))


class TickReader(Node):
    def __init__(self) -> None:
        super().__init__('measure_ticks')
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[int, int]] = None
        self._stamp = 0.0
        self._count = 0
        self.create_subscription(Int32MultiArray, ZEROED_TOPIC, self._on_zeroed, 10)
        self.reset_client = self.create_client(Trigger, RESET_SERVICE)

    def _on_zeroed(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 2:
            return
        with self._lock:
            self._latest = (int(msg.data[0]), int(msg.data[1]))
            self._stamp = time.monotonic()
            self._count += 1

    def snapshot(self) -> Tuple[Optional[Tuple[int, int]], float, int]:
        """(latest [left, right], age in seconds, number of messages received)."""
        with self._lock:
            age = time.monotonic() - self._stamp if self._latest is not None else float('inf')
            return self._latest, age, self._count

    def wait_for_messages(self, after_count: int, n: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.snapshot()[2] >= after_count + n:
                return True
            time.sleep(0.02)
        return False

    def reset(self) -> Tuple[bool, str]:
        if not self.reset_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            return False, f'{RESET_SERVICE} 서비스가 없음 (odometry 노드 실행 중인지 확인)'
        done = threading.Event()
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(SERVICE_TIMEOUT_S):
            return False, f'{RESET_SERVICE} 응답 없음 ({SERVICE_TIMEOUT_S}s)'
        res = future.result()
        if res is None:
            return False, f'{RESET_SERVICE} 호출 실패'
        return bool(res.success), res.message


def ask(prompt: str) -> str:
    """input() that turns Ctrl+D into a clean exit."""
    try:
        return input(prompt)
    except EOFError:
        print()
        raise KeyboardInterrupt


def measure_wheel(node: TickReader, wheel: tuple, turns: int) -> Optional[dict]:
    name, idx, label, invert_key = wheel
    other_idx = 1 - idx
    while True:
        print(f'\n=== {label} 바퀴 ({name}) ===')
        ok, message = node.reset()
        if not ok:
            print(f'[오류] 영점 실패: {message}')
            return None
        count_after_reset = node.snapshot()[2]
        if not node.wait_for_messages(count_after_reset, 2, 1.0):
            print(f'[오류] 영점 후 {ZEROED_TOPIC} 수신 없음 (STM32/base_controller 확인)')
            return None
        zero, _, _ = node.snapshot()
        print(f'영점 완료 ({message}). 현재값 [L, R] = {list(zero)}')
        if zero != (0, 0):
            print('  [주의] 영점 직후 값이 0이 아님 - 바퀴가 움직이는 중일 수 있음. 바퀴를 멈추고 다시 측정 권장')

        ask(f'{label} 바퀴를 앞으로만 정확히 {turns}바퀴 굴리고 Enter (다른 바퀴는 건드리지 말 것) ')
        time.sleep(SETTLE_S)
        value, age, _ = node.snapshot()
        if value is None or age > STALE_AFTER_S:
            print(f'[오류] {ZEROED_TOPIC} 마지막 수신이 {age:.1f}s 전 - 데이터가 끊김. '
                  '통신(base_controller 로그) 확인 후 다시 측정')
        else:
            result = evaluate(value, idx, other_idx, turns, label, invert_key)
            choice = ask('[Enter] 결과 채택  /  r + Enter: 이 바퀴 다시 측정  /  q + Enter: 종료 ')
            if choice.strip().lower() == 'q':
                raise KeyboardInterrupt
            if choice.strip().lower() != 'r':
                return result
            continue
        choice = ask('r + Enter: 다시 측정  /  q + Enter: 종료 ')
        if choice.strip().lower() == 'q':
            raise KeyboardInterrupt


def evaluate(value: Tuple[int, int], idx: int, other_idx: int, turns: int,
             label: str, invert_key: str) -> dict:
    measured = value[idx]
    other = value[other_idx]
    per_rev = measured / turns
    warnings: List[str] = []

    if measured == 0:
        warnings.append('카운트 0 - 엔코더 배선/전원 또는 바퀴가 실제로 돌았는지 확인')
    elif measured < 0:
        warnings.append(f'부호 음수 - 앞으로 굴렸다면 {invert_key} 설정 재확인 필요 '
                        '(enc_zeroed 는 invert 적용 후 값, 앞으로 = 양수가 정상)')
    if measured != 0 and abs(other) > CROSSTALK_RATIO * abs(measured):
        warnings.append(f'반대쪽 바퀴 값 {other} (측정 바퀴의 {abs(other) / abs(measured) * 100:.1f}%, '
                        f'기준 {CROSSTALK_RATIO * 100:.0f}%) - 다른 바퀴를 건드렸을 수 있음')
    elif measured == 0 and other != 0:
        warnings.append(f'반대쪽 바퀴 값 {other} - 바퀴를 잘못 굴렸거나 좌우 배선이 바뀌었을 수 있음')

    print(f'  {turns}바퀴 카운트 : {measured}   (반대쪽: {other})')
    print(f'  1바퀴당 카운트: {per_rev:.2f}')
    print(f'  부호 판정     : {"정상 (양수)" if measured > 0 else "비정상"}')
    for w in warnings:
        print(f'  [경고] {w}')
    return {'label': label, 'counts': measured, 'other': other,
            'per_rev': per_rev, 'ok': measured > 0, 'warnings': warnings}


def summarize(results: List[dict], turns: int) -> None:
    print('\n================ 결과 ================')
    print(f'{"바퀴":<6} {turns}바퀴 카운트   1바퀴당   반대쪽   판정')
    for r in results:
        verdict = '정상' if r['ok'] and not r['warnings'] else ('경고' if r['ok'] else '비정상')
        print(f'{r["label"]:<6} {r["counts"]:>12}   {r["per_rev"]:>8.2f}   {r["other"]:>6}   {verdict}')

    left, right = results
    if left['ok'] and right['ok']:
        tl, tr = round(left['per_rev']), round(right['per_rev'])
        diff = abs(left['per_rev'] - right['per_rev']) / max(left['per_rev'], right['per_rev']) * 100
        print(f'\n좌우 1바퀴당 차이: {diff:.1f}%')
        if any(r['warnings'] for r in results):
            print('[주의] 경고가 있음 - 적용 전에 재측정을 권장')
        print('적용 예시 (wheel_radius / wheel_separation 은 실측값으로 채워서 함께 넘기거나 생략):')
        print(f'  python3 ~/aircore_ws/src/aircore_base/scripts/apply_measurements.py '
              f'--ticks-left {tl} --ticks-right {tr}')
    else:
        print('\n[경고] 부호/카운트가 비정상인 바퀴가 있음 - invert_enc_* 와 배선을 확인하고 재측정. '
              '이 값은 적용하지 말 것.')


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='바퀴를 손으로 N바퀴 굴려 ticks_per_rev_left/right 를 측정한다. '
                    f'ROS 토픽({ZEROED_TOPIC})과 서비스({RESET_SERVICE})만 사용하며 '
                    '시리얼 포트를 열지 않고 /cmd_vel 도 publish 하지 않는다. '
                    'base_controller + odometry 가 실행 중이어야 한다.')
    parser.add_argument('--turns', type=int, default=5,
                        help='바퀴마다 굴릴 회전 수 (기본 5)')
    args = parser.parse_args(argv)
    if args.turns <= 0:
        parser.error('--turns 는 1 이상이어야 함')
    return args


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv if argv is None else argv
    args = parse_args(remove_ros_args(argv)[1:])

    # Ctrl+C -> plain KeyboardInterrupt in the main thread; rclpy must not tear the context
    # down underneath the spin thread.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = TickReader()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_spin = threading.Event()

    def spin() -> None:
        # Own loop (not executor.spin) so the thread can be stopped and joined before any
        # rclpy object is destroyed; destroying them mid-wait aborts the process.
        try:
            while not stop_spin.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        print(f'{ZEROED_TOPIC} 수신 대기 중...')
        if not node.wait_for_messages(0, 1, FIRST_MSG_TIMEOUT_S):
            print(f'[오류] {FIRST_MSG_TIMEOUT_S:.0f}s 동안 {ZEROED_TOPIC} 수신 없음.\n'
                  '  - base_controller/odometry 가 실행 중인지\n'
                  '  - 이 터미널도 ROS_DOMAIN_ID=87, ROS_LOCALHOST_ONLY=1 인지 확인')
            return 1
        print(f'수신 확인. 회전 수 = {args.turns}바퀴. 주의: reset_odom 은 odometry 위치도 0 으로 초기화함.')

        results = []
        for wheel in WHEELS:
            r = measure_wheel(node, wheel, args.turns)
            if r is None:
                return 1
            results.append(r)
        summarize(results, args.turns)
    except KeyboardInterrupt:
        print('\n중단됨.')
        exit_code = 130
    finally:
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
