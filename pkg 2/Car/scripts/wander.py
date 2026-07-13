#!/usr/bin/env python3
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Wander(Node):
    def __init__(self):
        super().__init__('wander')

        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.6)
        # 0.6 -> 0.8: 로봇 몸체 폭(약 16cm) 대비 여유가 너무 작아서 회전
        # 시작 시점에 이미 벽 모서리에 몸체가 닿을 수 있었음. 회전을 더
        # 일찍 시작해 실제 접촉(및 그로 인한 바퀴 미끄러짐/오도메트리
        # 오차)을 줄임.
        self.declare_parameter('safe_distance', 0.8)
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
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self._last_scan_time = None
        # 벽 앞에서 매 스캔마다 좌/우를 다시 비교하면 값이 거의 같을 때
        # 왼쪽<->오른쪽으로 방향이 계속 뒤집혀서 제자리에서 "와리가리"만
        # 하게 됨. 한 번 회전 방향을 정하면 앞이 확실히 트일 때까지(여유
        # 마진을 두고) 그 방향을 고수하도록 상태를 기억함.
        self._turning = False
        self._turn_dir = 1.0

        # 코너/좁은 틈에서는 회전만 반복하고 실제 위치는 안 바뀌는 경우가
        # 있음(회전해도 정면 각도가 조금씩만 바뀌어서 다시 걸리는 패턴).
        # 회전량이 아니라 오도메트리 실제 이동 거리로 "진짜 멈춤"을
        # 감지해서, 멈춰있으면 강제로 후진+큰 회전 탈출 기동을 함.
        self._odom_pos = None
        self._stuck_check_pos = None
        self._recovering_until = None
        self._recover_dir = 1.0
        self._latest_left = None
        self._latest_right = None
        self.create_timer(3.0, self._watchdog)
        self.get_logger().info('wander 시작. /scan 대기 중...')

    def on_odom(self, msg: Odometry):
        self._odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

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

        if self._odom_pos is not None:
            if self._stuck_check_pos is not None:
                dx = self._odom_pos[0] - self._stuck_check_pos[0]
                dy = self._odom_pos[1] - self._stuck_check_pos[1]
                moved = math.hypot(dx, dy)
                if moved < 0.08 and self._recovering_until is None:
                    self.get_logger().warn(
                        f'최근 3초간 {moved:.3f}m밖에 안 움직임(코너/좁은 틈에 '
                        '갇혔을 가능성) -> 탈출 기동 시작')
                    self._recovering_until = now + Duration(seconds=2.5)
                    # 방향을 무작위로 고르면 좁은 코너에 다시 걸릴 확률이
                    # 그대로라 갇힘->탈출->재갇힘이 반복됨. 후진하면서 더
                    # 트인 쪽(left/right 중 더 먼 쪽)으로 돌면 같은 코너로
                    # 재진입할 확률이 줄어듦.
                    if self._latest_left is not None and self._latest_right is not None:
                        self._recover_dir = (
                            1.0 if self._latest_left >= self._latest_right else -1.0)
                    else:
                        self._recover_dir = random.choice([-1.0, 1.0])
            self._stuck_check_pos = self._odom_pos

    def on_scan(self, msg: LaserScan):
        self._last_scan_time = self.get_clock().now()
        if not msg.ranges:
            return

        front = self._sector_min(msg, 0.0, self.front_half_angle)
        left = self._sector_min(msg, math.pi / 2, self.side_half_angle)
        right = self._sector_min(msg, -math.pi / 2, self.side_half_angle)
        self._latest_left = left
        self._latest_right = right

        twist = Twist()

        now = self.get_clock().now()
        if self._recovering_until is not None:
            if now < self._recovering_until:
                # 후진+회전을 동시에 명령하면 좁은 코너에서는 그 조합
                # 자체가 기하학적으로 불가능해서 로봇이 완전히 끼어버리고
                # (라이다 값이 전혀 안 바뀌는데도) 바퀴 관절만 헛돌아
                # 오도메트리가 폭주하는 게 실측으로 확인됨(ground truth
                # 대비 /odom이 7x7m 방에서 7m 가까이 벌어짐). 그래서 먼저
                # 순수 직진 후진으로 코너에서 몸부터 빼낸 다음에 그
                # 자리에서 회전하도록 두 단계로 분리함.
                remaining = (self._recovering_until - now).nanoseconds / 1e9
                if remaining > 1.3:
                    twist.linear.x = -0.1
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self._recover_dir * self.angular_speed
                self.get_logger().info('탈출 기동 중...', throttle_duration_sec=1.0)
                self.cmd_pub.publish(twist)
                return
            self._recovering_until = None
            self._turning = False

        # 회전 중이 아닐 때 벽을 만나면 회전을 "시작"하면서 방향을 한 번만
        # 정함. 회전 중일 때는 앞이 safe_distance보다 충분히(1.4배) 트일
        # 때까지 그 방향을 그대로 유지 — 안 그러면 살짝만 돌자마자 다시
        # 정면이 애매하게 걸려서 좌/우 판정이 뒤집히는 걸 반복하게 됨(와리가리).
        if not self._turning:
            if front < self.safe_distance:
                self._turning = True
                self._turn_dir = 1.0 if left >= right else -1.0
        else:
            if front > self.safe_distance * 1.4:
                self._turning = False

        if self._turning:
            # 양옆도 다 막혔으면(구석에 몰림) 살짝 후진하면서 회전
            if left < self.safe_distance and right < self.safe_distance:
                twist.linear.x = -0.05
            twist.angular.z = self._turn_dir * self.angular_speed
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