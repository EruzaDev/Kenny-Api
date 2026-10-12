"""Validate exported map destinations using only the Python standard library."""
import math


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def local_point(point, origin):
    x, y = point[0]-origin[0], point[1]-origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return c*x+s*y, -s*x+c*y


def destination_goal(env):
    marker = next(m for m in env['markers'] if m['id'] == env['destination_marker_id'])
    offset = .5 if marker['mount'] == 'wall' else 0.
    return [marker['x']+math.cos(marker['yaw'])*offset,
            marker['y']+math.sin(marker['yaw'])*offset]


def validate_environment(env):
    if not isinstance(env, dict) or (env.get('version'), env.get('frame_id'), env.get('units')) != (1, 'map', 'meters'):
        raise ValueError('Expected exported version 1, frame_id map, and units meters')
    limit = {'DICT_4X4_50': 50, 'DICT_4X4_1000': 1000}.get(env.get('dictionary'))
    if limit is None:
        raise ValueError('Supported dictionaries: DICT_4X4_50, DICT_4X4_1000')
    grid = env['grid']
    w, h, res = grid['width'], grid['height'], grid['resolution']
    if type(w) is not int or type(h) is not int or not (1 <= w <= 4000 and 1 <= h <= 4000) or w*h > 4_000_000:
        raise ValueError('Invalid grid dimensions (maximum four million cells)')
    if not number(res) or not .005 <= res <= 1:
        raise ValueError('Resolution must be 0.005–1 meters')
    origin = grid['origin']
    if not isinstance(origin, list) or len(origin) != 3 or not all(map(number, origin)):
        raise ValueError('Grid origin must contain finite x, y, yaw')
    data = grid['data']
    if not isinstance(data, list) or len(data) != w*h or any(type(v) is not int or v not in (-1, 0, 100) for v in data):
        raise ValueError('Grid must contain width*height occupancy values: -1, 0, 100')
    markers = env.get('markers')
    if not isinstance(markers, list) or not markers:
        raise ValueError('Export must contain at least one surveyed marker')
    ids = set()
    for marker in markers:
        mid = marker['id']
        if type(mid) is not int or not 0 <= mid < limit or mid in ids:
            raise ValueError('Marker IDs must be unique integers within the dictionary range')
        ids.add(mid)
        if not all(number(marker[k]) for k in ('x', 'y', 'z', 'yaw', 'size')) or marker['mount'] not in ('wall', 'floor') or marker['z'] < 0 or not .01 <= marker['size'] <= 1:
            raise ValueError('Invalid marker dimensions, pose, or mounting')
        x, y = local_point([marker['x'], marker['y']], origin)
        if not 0 <= x <= w*res or not 0 <= y <= h*res:
            raise ValueError('Marker lies outside the map')
    for obstacle in env.get('obstacles', []):
        if obstacle['kind'] not in ('wall', 'obstacle', 'prohibited') or not all(number(obstacle[k]) for k in ('x', 'y', 'width', 'depth', 'bottom', 'height')):
            raise ValueError('Invalid obstacle kind or dimensions')
        if min(obstacle['width'], obstacle['depth'], obstacle['height']) <= 0 or obstacle['bottom'] < 0:
            raise ValueError('Obstacle dimensions must be positive')
    return env


def arrival_is_free(env, goal):
    grid = env['grid']
    res = grid['resolution']
    x, y = local_point(goal, grid['origin'])
    col, row = math.floor(x/res), math.floor(y/res)
    if not (0 <= col < grid['width'] and 0 <= row < grid['height']):
        return False
    if grid['data'][row*grid['width']+col] != 0:
        return False
    # Match the editor rasterizer: test the cell center in ROS world space,
    # including half a cell of obstacle padding when stamping rectangles.
    lx, ly = (col+.5)*res, (row+.5)*res
    ox, oy, yaw = grid['origin']
    wx = ox+math.cos(yaw)*lx-math.sin(yaw)*ly
    wy = oy+math.sin(yaw)*lx+math.cos(yaw)*ly
    return not any(o['x']-res/2 <= wx <= o['x']+o['width']+res/2 and
                   o['y']-res/2 <= wy <= o['y']+o['depth']+res/2
                   for o in env.get('obstacles', []))
