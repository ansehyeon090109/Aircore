"""Aircore base controller: STM32 serial bridge + differential drive motor controller.

Protocol (fixed):
  Jetson -> STM32 : "M,left,right\\n"   (int duty, -100..100)
  STM32 -> Jetson : "ENC,left,right\\n" (cumulative encoder counts, 50 ms period)
STM32 stops the motors if UART is silent for >300 ms, so we always keep sending.
"""
import math
import signal
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

RECONNECT_PERIOD_S = 1.0
SHUTDOWN_STOP_REPEAT = 5


def round_half_away(x: float) -> int:
    """Symmetric rounding (Python round() is banker's rounding)."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def to_int32(value: int) -> int:
    """STM32 sends counts as uint32; reinterpret as signed int32 (e.g. 4294967283 -> -13)."""
    if not -2**31 <= value < 2**32:  # already-signed values pass through unchanged
        raise ValueError(f'out of int32/uint32 range: {value}')
    return value - 2**32 if value >= 2**31 else value


class BaseController(Node):
    def __init__(self) -> None:
        super().__init__('base_controller')

        self.port: str = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.baudrate: int = self.declare_parameter('baudrate', 115200).value
        self.send_rate_hz: float = float(self.declare_parameter('send_rate_hz', 20.0).value)
        self.cmd_vel_timeout: float = float(self.declare_parameter('cmd_vel_timeout', 0.5).value)
        self.wheel_separation: float = float(self.declare_parameter('wheel_separation', 0.0).value)
        # Not used here; declared for the upcoming odometry stage.
        self.wheel_radius: float = float(self.declare_parameter('wheel_radius', 0.0).value)
        self.ticks_per_rev: int = self.declare_parameter('ticks_per_rev', 0).value
        self.max_wheel_speed: float = float(self.declare_parameter('max_wheel_speed', 0.0).value)
        self.invert_left: bool = self.declare_parameter('invert_left', False).value
        self.invert_right: bool = self.declare_parameter('invert_right', False).value
        self.max_duty: int = max(0, min(100, int(self.declare_parameter('max_duty', 30).value)))
        self.dry_run: bool = self.declare_parameter('dry_run', False).value

        if self.send_rate_hz <= 0.0:
            self.get_logger().error(f'send_rate_hz={self.send_rate_hz} invalid, using 20.0')
            self.send_rate_hz = 20.0

        self.configured = self.wheel_separation > 0.0 and self.max_wheel_speed > 0.0
        self._unconfigured_warned = False

        # Latest motor command (already converted to duty) and when cmd_vel arrived.
        self._cmd_lock = threading.Lock()
        self._duty_l = 0
        self._duty_r = 0
        self._last_cmd_time: Optional[float] = None

        # Serial port is shared between the reader thread and the send timer.
        self._ser_lock = threading.Lock()
        self._ser: Optional[serial.Serial] = None
        self._running = True

        self.enc_pub = self.create_publisher(Int32MultiArray, '/aircore/enc_raw', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self.get_logger().info(
            f'port={self.port} baud={self.baudrate} rate={self.send_rate_hz}Hz '
            f'timeout={self.cmd_vel_timeout}s L={self.wheel_separation} '
            f'max_wheel_speed={self.max_wheel_speed} max_duty={self.max_duty} '
            f'invert=({self.invert_left},{self.invert_right}) dry_run={self.dry_run}')
        if not self.configured:
            self.get_logger().warn(
                'wheel_separation / max_wheel_speed 미설정(0.0) → M,0,0만 송신합니다.')

        if self.dry_run:
            self.get_logger().warn('dry_run=true: 시리얼 포트를 열지 않고 송신 문자열만 로그합니다.')
        else:
            self._reader = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader.start()

        # Stop command first, then periodic send (keep-alive).
        self._send(0, 0)
        self.create_timer(1.0 / self.send_rate_hz, self._on_send_timer)

    # ---------------- Motor controller ----------------

    def _on_cmd_vel(self, msg: Twist) -> None:
        v = msg.linear.x
        w = msg.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn('non-finite cmd_vel ignored', throttle_duration_sec=2.0)
            v, w = 0.0, 0.0

        if not self.configured:
            if not self._unconfigured_warned and (v != 0.0 or w != 0.0):
                self.get_logger().warn(
                    'cmd_vel 수신했지만 wheel_separation/max_wheel_speed 미설정 → M,0,0 유지')
                self._unconfigured_warned = True
            duty_l, duty_r = 0, 0
        else:
            v_l = v - w * self.wheel_separation / 2.0
            v_r = v + w * self.wheel_separation / 2.0
            duty_l = self._to_duty(v_l, self.invert_left)
            duty_r = self._to_duty(v_r, self.invert_right)
            self.get_logger().info(
                f'cmd_vel v={v:.3f} w={w:.3f} -> v_l={v_l:.3f} v_r={v_r:.3f} m/s '
                f'-> duty L={duty_l} R={duty_r}',
                throttle_duration_sec=0.5)

        with self._cmd_lock:
            self._duty_l, self._duty_r = duty_l, duty_r
            self._last_cmd_time = time.monotonic()

    def _to_duty(self, v_wheel: float, invert: bool) -> int:
        duty = round_half_away(v_wheel / self.max_wheel_speed * 100.0)
        duty = max(-self.max_duty, min(self.max_duty, duty))
        return -duty if invert else duty

    def _on_send_timer(self) -> None:
        with self._cmd_lock:
            fresh = (self._last_cmd_time is not None
                     and time.monotonic() - self._last_cmd_time <= self.cmd_vel_timeout)
            duty_l, duty_r = (self._duty_l, self._duty_r) if fresh else (0, 0)
        self._send(duty_l, duty_r)

    def _send(self, duty_l: int, duty_r: int) -> None:
        line = f'M,{duty_l},{duty_r}\n'
        if self.dry_run:
            self.get_logger().info(f'[dry_run] {line.strip()}')
            return
        with self._ser_lock:
            if self._ser is None:
                return
            try:
                self._ser.write(line.encode('ascii'))
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'serial write failed: {e}')
                self._close_port_locked()

    # ---------------- Serial bridge ----------------

    def _reader_loop(self) -> None:
        buf = b''
        while self._running:
            with self._ser_lock:
                ser = self._ser
            if ser is None:
                if self._open_port():
                    buf = b''
                    self._send(0, 0)  # stop first right after (re)connect
                else:
                    time.sleep(RECONNECT_PERIOD_S)
                continue
            try:
                chunk = ser.read(ser.in_waiting or 1)  # blocks up to timeout
            except (serial.SerialException, OSError, TypeError) as e:
                if self._running:
                    self.get_logger().error(f'serial read failed: {e}')
                with self._ser_lock:
                    self._close_port_locked()
                continue
            buf += chunk
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._handle_line(raw)
            if len(buf) > 256:  # garbage without newline
                buf = b''

    def _handle_line(self, raw: bytes) -> None:
        line = raw.decode('ascii', errors='replace').strip()
        if not line:
            return
        parts = line.split(',')
        try:
            if len(parts) != 3 or parts[0] != 'ENC':
                raise ValueError
            left, right = to_int32(int(parts[1])), to_int32(int(parts[2]))
        except ValueError:
            self.get_logger().warn(f'malformed line ignored: {line!r}', throttle_duration_sec=2.0)
            return
        msg = Int32MultiArray()
        msg.data = [left, right]
        self.enc_pub.publish(msg)

    def _open_port(self) -> bool:
        try:
            ser = serial.Serial(self.port, self.baudrate, bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                timeout=0.1, write_timeout=0.1,
                                xonxoff=False, rtscts=False, dsrdtr=False)
            ser.reset_input_buffer()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'cannot open {self.port}: {e} (retrying)',
                                   throttle_duration_sec=5.0)
            return False
        with self._ser_lock:
            self._ser = ser
        self.get_logger().info(f'serial connected: {self.port} @ {self.baudrate}')
        return True

    def _close_port_locked(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def shutdown(self) -> None:
        """Send stop a few times, then close the port."""
        # Ctrl+C under launch delivers SIGINT twice; don't let the 2nd one cut this short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for _ in range(SHUTDOWN_STOP_REPEAT):
            self._send(0, 0)
            time.sleep(0.02)
        self._running = False
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.flush()
                except Exception:
                    pass
            self._close_port_locked()


def _raise_interrupt(signum, frame) -> None:
    raise KeyboardInterrupt


def main(args: Optional[list] = None) -> None:
    # Own the signals so the context stays valid until M,0,0 has been sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = BaseController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
