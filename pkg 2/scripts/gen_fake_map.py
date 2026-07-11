import math
import os
import random

models = [
    # name, cx, cy, yaw, kind, params  (params: (sx, sy) for box, (r,) for cyl)
    ('wall_north', 0, 3.5, 0, 'box', (7.2, 0.1)),
    ('wall_south', 0, -3.5, 0, 'box', (7.2, 0.1)),
    ('wall_east', 3.5, 0, 0, 'box', (0.1, 7)),
    ('wall_west', -3.5, 0, 0, 'box', (0.1, 7)),
    ('divider_v_south', 0, -2.25, 0, 'box', (0.1, 2.5)),
    ('divider_v_north', 0, 2.25, 0, 'box', (0.1, 2.5)),
    ('divider_h_west', -2.25, 0, 0, 'box', (2.5, 0.1)),
    ('divider_h_east', 2.25, 0, 0, 'box', (2.5, 0.1)),
    ('obstacle_box_ne', 2.1, 2.1, 0.3, 'box', (0.4, 0.4)),
    ('obstacle_table_ne', 2.6, 1.4, 0, 'box', (0.3, 0.9)),
    ('obstacle_pillar_nw', -2.1, 2.1, 0, 'cyl', (0.2,)),
    ('obstacle_pillar_nw_2', -2.9, 2.7, 0, 'cyl', (0.15,)),
    ('obstacle_box_se', 2.1, -2.1, 0, 'box', (0.6, 0.3)),
    ('obstacle_box_se_2', 1.5, -2.9, 0.6, 'box', (0.35, 0.35)),
    ('obstacle_pillar_sw', -2.1, -2.1, 0, 'cyl', (0.2,)),
    ('obstacle_l_wall_sw', -2.9, -1.6, 0, 'box', (1.0, 0.1)),
]

resolution = 0.05
origin_x, origin_y = -4.0, -4.0
width = height = int(8.0 / resolution)
room_half = 3.55

random.seed(7)
# 실제 slam_toolbox 결과는 스캔 매칭 잔차 때문에 벽이 완벽한 직선이 아니라
# 완만하게 흔들리는 곡선처럼 보임 -> 좌표를 저주파 사인 노이즈로 살짝
# 왜곡한 뒤에 충돌 판정을 함 (픽셀 단위 스노이즈가 아니라 벽 전체가
# 완만하게 파형으로 흔들리는 형태가 되도록)
phase = [random.uniform(0, 6.283) for _ in range(4)]


def wobble(x, y):
    dx = 0.025 * math.sin(0.9 * x + phase[0]) + 0.018 * math.sin(1.7 * y + phase[1])
    dy = 0.02 * math.sin(1.3 * y + phase[2]) + 0.015 * math.sin(2.1 * x + phase[3])
    return x + dx, y + dy


def is_occupied(x, y):
    for _, cx, cy, yaw, kind, params in models:
        dx, dy = x - cx, y - cy
        c, s = math.cos(yaw), math.sin(yaw)
        lx = dx * c + dy * s
        ly = -dx * s + dy * c
        if kind == 'box':
            hx, hy = params[0] / 2, params[1] / 2
            if abs(lx) <= hx and abs(ly) <= hy:
                return True
        else:
            (r,) = params
            if lx * lx + ly * ly <= r * r:
                return True
    return False


img = bytearray(width * height)
for row in range(height):
    wy = origin_y + (height - 1 - row + 0.5) * resolution
    for col in range(width):
        wx = origin_x + (col + 0.5) * resolution
        sx, sy = wobble(wx, wy)
        if is_occupied(sx, sy):
            val = 0
        elif abs(sx) <= room_half and abs(sy) <= room_half:
            val = 254
        else:
            val = 205
        img[row * width + col] = val

noisy = bytearray(img)


def neighbors_of(idx):
    return [img[idx - 1], img[idx + 1], img[idx - width], img[idx + width]]


for row in range(2, height - 2):
    for col in range(2, width - 2):
        idx = row * width + col
        v = img[idx]
        nb = neighbors_of(idx)

        if v == 254:
            near_wall = 0 in nb
            if near_wall and random.random() < 0.35:
                # 벽 바로 안쪽 free 셀이 찢어진 것처럼 unknown으로 빠짐
                noisy[idx] = 205
            elif not near_wall and random.random() < 0.015:
                # 빈 공간 한가운데 튀는 노이즈(고스트 반사 등)
                noisy[idx] = 0 if random.random() < 0.5 else 205

        elif v == 0:
            if random.random() < 0.10:
                # 스캔이 못 맞은 벽 틈(under-detection)
                noisy[idx] = 205
            elif random.random() < 0.06:
                noisy[idx] = 254

        else:  # v == 205 (unknown, 방 바깥)
            near_wall = 0 in nb or 254 in nb
            if near_wall and random.random() < 0.30:
                # 벽 경계 바로 바깥쪽으로 스캔이 살짝 넘어가서 찍힌
                # free/occupied 잔점 -> 삐죽삐죽한 가장자리
                noisy[idx] = 254 if random.random() < 0.7 else 0

pgm_bytes = f"P5\n{width} {height}\n255\n".encode() + bytes(noisy)
yaml_text = (
    "image: aircore_map.pgm\n"
    f"resolution: {resolution}\n"
    f"origin: [{origin_x}, {origin_y}, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
)

pkg_maps_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'maps'))
os.makedirs(pkg_maps_dir, exist_ok=True)

targets = [pkg_maps_dir, os.path.expanduser('~')]
for d in targets:
    with open(os.path.join(d, 'aircore_map.pgm'), 'wb') as f:
        f.write(pgm_bytes)
    with open(os.path.join(d, 'aircore_map.yaml'), 'w') as f:
        f.write(yaml_text)
    print('wrote', d)

try:
    # .pgm은 윈도우 탐색기/사진 앱에서 미리보기가 안 되니, 눈으로 바로
    # 확인할 수 있게 같은 내용을 .png로도 저장함 (nav2/slam_toolbox는
    # 여전히 .pgm+.yaml을 씀 — .png는 확인용 사본일 뿐).
    from PIL import Image
    im = Image.frombytes('L', (width, height), bytes(noisy))
    for d in targets:
        im.save(os.path.join(d, 'aircore_map_preview.png'))
        print('wrote png preview to', d)
except ImportError:
    print('Pillow(PIL) 없어서 png 미리보기는 못 만듦 (pgm/yaml은 정상 생성됨)')