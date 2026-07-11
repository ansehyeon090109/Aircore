#!/usr/bin/env python3
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Wander(Node):
    def __init__(self):
        super().__init__('wander')

        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('safe_distance', 0.6)
        self.declare_parameter('front_half_angle_deg', 35.0)
        self.declare_parameter('side_half_angle_deg', 40.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value)
        self.side_half_angle = math.radians(
            self.get_parameter('side_half_angle_deg').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self._last_scan_time = None
        self.create_timer(3.0, self._watchdog)
        self.get_logger().info('wander 시작. /scan 대기 중...')

    @staticmethod
    def _sector_min(msg, center_angle, half_width):
        lo = center_angle - half_width
        hi = center_angle + half_width
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            a = msg.angle_min + i * msg.angle_increment
            if lo <= a <= hi and msg.range_min < r < msg.range_max:
                best = min(best, r)
        return best

    def _watchdog(self):
        # /scan이 안 들어오면 여기서 원인을 바로 알 수 있게 경고를 남김 —
        # 그렇지 않으면 "아무 반응 없음"만 보이고 왜인지 알 방법이 없었음
        now = self.get_clock().now()
        if self._last_scan_time is None:
            self.get_logger().warn(
                '아직 /scan을 한 번도 못 받음 — gazebo.launch.py가 켜져 있는지, '
                '/scan 토픽이 실제로 발행되는지(ros2 topic hz /scan) 확인하세요.')
            return
        age = (now - self._last_scan_time).nanoseconds / 1e9
        if age > 3.0:
            self.get_logger().warn(
                f'/scan이 {age:.1f}초째 안 들어옴 — 브릿지/센서가 죽었을 수 있습니다.')

    def on_scan(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()
        if not msg.ranges:
            return

        front = self._sector_min(msg, 0.0, self.front_half_angle)
        left = self._sector_min(msg, math.pi / 2, self.side_half_angle)
        right = self._sector_min(msg, -math.pi / 2, self.side_half_angle)

        twist = Twist()

        if front < self.safe_distance:
            # 앞이 막힘 -> 더 트인 쪽으로 제자리 회전. 양옆도 다 막혔으면 살짝 후진.
            if left < self.safe_distance and right < self.safe_distance:
                twist.linear.x = -0.05
            twist.angular.z = self.angular_speed if left > right else -self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            # 직선으로만 계속 가면 좁고 긴 궤적만 스캔되니, 약한 무작위 회전으로
            # 넓은 영역을 지나가게 함
            twist.angular.z = random.uniform(-0.15, 0.15)

        self.get_logger().info(
            f'front={front:.2f} left={left:.2f} right={right:.2f} '
            f'-> v={twist.linear.x:.2f} w={twist.angular.z:.2f}',
            throttle_duration_sec=2.0)

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = Wander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()