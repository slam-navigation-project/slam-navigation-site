"""
맵 창에서 쓰는 순수 계산 모듈 (tkinter / paramiko / numpy 없이 동작)

- yaml 최소 파서 (PC에 PyYAML이 없어도 됨)
- 월드(m) <-> 캔버스(픽셀) 좌표 변환
- 클릭 위치에서 가장 가까운 웨이포인트 찾기
- 파이 노드와 주고받는 JSON 명령/상태 처리
"""
import json
import math
import time

CMD_FILE = "/tmp/amr_cmd.json"
STATE_FILE = "/tmp/amr_state.json"


def read_pgm_size(path):
    """PGM(P5) 헤더에서 (너비, 높이)만 읽는다."""
    with open(path, "rb") as f:
        head = f.read(64)
    tokens, pos = [], 0
    while len(tokens) < 3:
        while head[pos:pos + 1].isspace():
            pos += 1
        if head[pos:pos + 1] == b"#":
            while head[pos:pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        end = pos
        while end < len(head) and not head[end:end + 1].isspace():
            end += 1
        tokens.append(head[pos:end])
        pos = end
    if tokens[0] != b"P5":
        raise ValueError("P5 형식 PGM만 지원합니다")
    return int(tokens[1]), int(tokens[2])


def parse_map_yaml(text):
    """room_map.yaml 에서 image, resolution, origin 을 꺼낸다."""
    out = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, val = [s.strip() for s in line.split(":", 1)]
        out[key] = val
    image = out["image"].strip("'\"")
    resolution = float(out["resolution"])
    origin = [float(v) for v in out["origin"].strip("[]").split(",")]
    return {"image": image, "resolution": resolution, "origin": origin[:2]}


class MapView:
    """맵 이미지(픽셀 w x h)를 scale배로 그린 캔버스와 map 프레임(m) 사이의 좌표 변환."""

    def __init__(self, width, height, resolution, origin, scale):
        self.w, self.h = width, height
        self.res = resolution
        self.ox, self.oy = origin
        self.s = scale

    @property
    def canvas_size(self):
        return self.w * self.s, self.h * self.s

    def world_to_canvas(self, x, y):
        col = (x - self.ox) / self.res
        row = (y - self.oy) / self.res
        return col * self.s, (self.h - row) * self.s      # 이미지는 위가 북쪽이라 y를 뒤집는다

    def canvas_to_world(self, px, py):
        col = px / self.s
        row = self.h - py / self.s
        return self.ox + col * self.res, self.oy + row * self.res

    def inside(self, px, py):
        return 0 <= px < self.w * self.s and 0 <= py < self.h * self.s

    def nearest_point(self, points, px, py, max_px=18):
        """클릭한 캔버스 좌표에서 가장 가까운 웨이포인트의 번호. 너무 멀면 None."""
        best, best_d = None, max_px
        for i, (x, y) in enumerate(points):
            cx, cy = self.world_to_canvas(x, y)
            d = math.hypot(cx - px, cy - py)
            if d <= best_d:
                best, best_d = i, d
        return best

    def robot_polygon(self, x, y, yaw, size=0.12):
        """로봇을 삼각형으로 그리기 위한 캔버스 좌표 6개 값 (x1,y1,...)."""
        pts = []
        for fx, fy in ((size, 0.0), (-size * 0.7, size * 0.6), (-size * 0.7, -size * 0.6)):
            wx = x + fx * math.cos(yaw) - fy * math.sin(yaw)
            wy = y + fx * math.sin(yaw) + fy * math.cos(yaw)
            pts.extend(self.world_to_canvas(wx, wy))
        return pts


def make_command(kind, points=None):
    """파이 노드가 읽는 명령 JSON 문자열. seq는 매번 달라야 새 명령으로 인식된다."""
    cmd = {"seq": int(time.time() * 1000), "cmd": kind}
    if points is not None:
        cmd["points"] = [[round(float(x), 3), round(float(y), 3)] for x, y in points]
    return json.dumps(cmd)


def parse_state(text):
    """파이 상태 JSON을 읽는다. 깨졌거나 비어 있으면 None."""
    try:
        s = json.loads(text)
    except (TypeError, ValueError):
        return None
    return s if isinstance(s, dict) else None
