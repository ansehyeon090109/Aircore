# Grab /scan and summarize near returns (0.2-0.7 m) in laser frame and in base_footprint
# (laser at x=+0.045 with yaw=pi: x_b = 0.045 - x_l, y_b = -y_l).
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

LASER_X, LASER_YAW = 0.045, math.pi
msgs = []


class Probe(Node):
    def __init__(self):
        super().__init__('scan_probe')
        self.create_subscription(LaserScan, '/scan', msgs.append, qos_profile_sensor_data)


rclpy.init()
node = Probe()
while len(msgs) < 5 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=1.0)
m = msgs[-1]
pts = [(math.degrees(m.angle_min + i * m.angle_increment), r) for i, r in enumerate(m.ranges)]
valid = [(a, r) for a, r in pts if math.isfinite(r) and m.range_min <= r <= m.range_max]
print(f'frame={m.header.frame_id} points={len(pts)} valid={len(valid)}')

near = sorted((a, r) for a, r in valid if 0.2 <= r <= 0.7)
clusters, cur = [], []
for a, r in near:
    if cur and a - cur[-1][0] > 3.0:
        clusters.append(cur)
        cur = []
    cur.append((a, r))
if cur:
    clusters.append(cur)
if len(clusters) > 1 and clusters[0][0][0] + 180 < 3 and 180 - clusters[-1][-1][0] < 3:
    clusters[0] = clusters.pop() + clusters[0]

print('near clusters (0.2-0.7 m from laser), largest first:')
for c in sorted(clusters, key=len, reverse=True)[:6]:
    xl = sum(r * math.cos(math.radians(a)) for a, r in c) / len(c)
    yl = sum(r * math.sin(math.radians(a)) for a, r in c) / len(c)
    xb = LASER_X + xl * math.cos(LASER_YAW) - yl * math.sin(LASER_YAW)
    yb = xl * math.sin(LASER_YAW) + yl * math.cos(LASER_YAW)
    side = ('FRONT' if xb > 0.1 else 'REAR' if xb < -0.1 else 'SIDE') + (' left' if yb > 0.1 else ' right' if yb < -0.1 else '')
    print(f'  {len(c):3d} pts  laser {c[0][0]:+6.1f}..{c[-1][0]:+6.1f} deg {min(r for _, r in c):.2f}-{max(r for _, r in c):.2f} m'
          f'  | base_footprint x={xb:+.2f} y={yb:+.2f} -> {side}')
node.destroy_node()
rclpy.shutdown()
