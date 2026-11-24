import argparse
import time

import serial


def signed_delta(current, previous):
    return ((current - previous + 0x80000000) & 0xFFFFFFFF) - 0x80000000


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--left', type=int, default=0, choices=range(-100, 101))
    parser.add_argument('--right', type=int, default=0, choices=range(-100, 101))
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--timeout-test', action='store_true')
    args = parser.parse_args()
    previous = None
    command = f'M,{args.left},{args.right}\n'.encode('ascii')
    with serial.Serial(args.port, 115200, timeout=0.02) as uart:
        start = time.monotonic()
        next_send = start
        end = start + max(0, args.duration)
        monitor_end = end + (0.6 if args.timeout_test else 0)
        try:
            while time.monotonic() < monitor_end:
                now = time.monotonic()
                if now < end and now >= next_send:
                    uart.write(command)
                    next_send = now + 0.05
                line = uart.readline().decode('ascii', errors='replace').strip()
                if not line:
                    continue
                fields = line.split(',')
                if len(fields) == 3 and fields[0] == 'ENC':
                    try:
                        current = (int(fields[1]), int(fields[2]))
                    except ValueError:
                        print('Invalid encoder line:', line)
                        continue
                    delta = None if previous is None else tuple(
                        signed_delta(c, p) for c, p in zip(current, previous)
                    )
                    print(f'{now - start:.3f}s {line} delta={delta}')
                    previous = current
                else:
                    print('RX:', line)
        finally:
            uart.write(b'M,0,0\n')
            uart.flush()


if __name__ == '__main__':
    main()
