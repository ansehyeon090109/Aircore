"""Aircore wheel odometry: /aircore/enc_raw -> /odom + TF(odom -> base_link).

Input : /aircore/enc_raw (std_msgs/Int32MultiArray, data=[left, right]),
        cumulative encoder counts already converted to signed int32 by base_controller.
Output: /odom (nav_msgs/Odometry) and TF odom -> base_link.

Differential drive, midpoint integration:
  d_L = dn_L / ticks_per_rev_left  * 2*pi*wheel_radius   (same for right)
  d = (d_R + d_L) / 2,  dtheta = (d_R - d_L) / wheel_separation
  x += d*cos(theta + dtheta/2), y += d*sin(theta + dtheta/2), theta += dtheta
"""
import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# Conservative diagonal covariances (x, y, z, roll, pitch, yaw); 1e6 = not observed (2D robot).
POSE_COV_DIAG = (0.01, 0.01, 1e6, 1e6, 1e6, 0.03)
TWIST_COV_DIAG = (0.01, 1e6, 1e6, 1e6, 1e6, 0.03)


def diag6(values) -> list:
    """6 diagonal values -> row-major 6x6 covariance (36 floats)."""
    cov = [0.0] * 36
    for i, v in enumerate(values):
        cov[i * 6 + i] = float(v)
    return cov


def wrap_int32_diff(cur: int, prev: int) -> int:
    """Count difference across int32 wraparound (e.g. 2147483647 -> -2147483648 gives +1)."""
    dn = cur - prev
    if dn >= 2**31:
        dn -= 2**32
    elif dn < -2**31:
        dn += 2**32
    return dn


def wrap_to_int32(v: int) -> int:
    """Wrap an arbitrary Python int into the int32 range."""
    return ((v + 2**31) % 2**32) - 2**31


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('odometry')

        # Unmeasured values default to 0 (= unset); nothing is published until all are set.
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.0).value)
        self.wheel_separation = float(self.declare_parameter('wheel_separation', 0.0).value)
        self.ticks_left = int(self.declare_parameter('ticks_per_rev_left', 0).value)
        self.ticks_right = int(self.declare_parameter('ticks_per_rev_right', 0).value)
        # Left encoder was seen counting down when driving forward (needs re-check).
        self.invert_left = bool(self.declare_parameter('invert_enc_left', True).value)
        self.invert_right = bool(self.declare_parameter('invert_enc_right', False).value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        self.configured = (self.wheel_radius > 0.0 and self.wheel_separation > 0.0
                           and self.ticks_left > 0 and self.ticks_right > 0)
        self._unconfigured_warned = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_counts: Optional[tuple] = None
        self._prev_time: Optional[Time] = None
        # Counts since last zero, direction-corrected (forward = increasing), unbounded Python ints.
        self._zeroed = [0, 0]

        self.zeroed_pub = self.create_publisher(Int32MultiArray, '/aircore/enc_zeroed', 10)
        self.create_service(Trigger, '/aircore/reset_odom', self._on_reset)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Int32MultiArray, '/aircore/enc_raw', self._on_enc, 10)

        self.get_logger().info(
            f'r={self.wheel_radius} L={self.wheel_separation} '
            f'ticks=({self.ticks_left},{self.ticks_right}) '
            f'invert=({self.invert_left},{self.invert_right}) '
            f'frames={self.odom_frame}->{self.base_frame} publish_tf={self.publish_tf}')

    def _on_reset(self, request, response):
        if self._prev_counts is None:
            response.success = False
            response.message = 'no encoder message received yet; cannot reset'
            return response
        self.x = self.y = self.theta = 0.0
        self._zeroed = [0, 0]
        # _prev_counts already holds the last raw counts -> they become the new reference.
        self._prev_time = self.get_clock().now()
        self._publish_zeroed()
        response.success = True
        response.message = 'reset to 0,0'
        self.get_logger().info('odometry reset: pose=0, counts=[0,0]')
        return response

    def _publish_zeroed(self) -> None:
        out = Int32MultiArray()
        out.data = [wrap_to_int32(self._zeroed[0]), wrap_to_int32(self._zeroed[1])]
        self.zeroed_pub.publish(out)

    def _on_enc(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 2:
            self.get_logger().warn(f'enc_raw needs 2 values, got {len(msg.data)}: ignored',
                                   throttle_duration_sec=2.0)
            return

        now = self.get_clock().now()
        left, right = int(msg.data[0]), int(msg.data[1])

        if self._prev_counts is None:
            # First message: store the reference only.
            self._prev_counts = (left, right)
            self._prev_time = now
            self.get_logger().info(f'reference counts set: L={left} R={right}')
            self._publish_zeroed()
            return

        dn_l = wrap_int32_diff(left, self._prev_counts[0])
        dn_r = wrap_int32_diff(right, self._prev_counts[1])
        if self.invert_left:
            dn_l = -dn_l
        if self.invert_right:
            dn_r = -dn_r

        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_counts = (left, right)
        self._prev_time = now

        self._zeroed[0] += dn_l
        self._zeroed[1] += dn_r
        self._publish_zeroed()

        if not self.configured:
            if not self._unconfigured_warned:
                self.get_logger().warn(
                    'wheel_radius / wheel_separation / ticks_per_rev_left / ticks_per_rev_right '
                    '미설정(0) → /odom, TF를 publish하지 않습니다 (/aircore/enc_zeroed는 publish).')
                self._unconfigured_warned = True
            return

        circumference = 2.0 * math.pi * self.wheel_radius
        d_l = dn_l / self.ticks_left * circumference
        d_r = dn_r / self.ticks_right * circumference
        d = (d_r + d_l) / 2.0
        dtheta = (d_r - d_l) / self.wheel_separation

        mid = self.theta + dtheta / 2.0
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.theta = normalize_angle(self.theta + dtheta)

        if dt > 0.0:
            v, w = d / dt, dtheta / dt
        else:
            v, w = 0.0, 0.0
            self.get_logger().warn(f'non-positive dt={dt:.6f}s: velocity set to 0',
                                   throttle_duration_sec=2.0)

        self._publish(now, v, w)

    def _publish(self, now: Time, v: float, w: float) -> None:
        stamp = now.to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = diag6(POSE_COV_DIAG)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.twist.covariance = diag6(TWIST_COV_DIAG)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
