#!/usr/bin/env python3
"""
A* 경로계획 모듈 (ROS 없이 단독 동작)

- slam_toolbox가 저장한 room_map.yaml + room_map.pgm 을 읽는다.
- 미탐색(회색)과 장애물(검정)을 모두 막힌 칸으로 보고, 차 반경만큼 부풀린다(inflation).
- 8방향 A*로 경로를 찾고, 일직선으로 갈 수 있는 구간은 합쳐서 꼭짓점만 남긴다.

좌표계: 'map' 프레임(미터). x는 오른쪽, y는 위쪽. (PGM 이미지는 위아래가 뒤집혀 있어 내부에서 변환)
"""
import heapq
import math
import os

import numpy as np
import yaml
from astar import astar as astar_4way


def read_pgm(path):
    """PGM(P5, 바이너리)을 numpy 배열(행=이미지 위에서 아래)로 읽는다. PIL이 없는 파이에서도 동작."""
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                      # 매직, 너비, 높이, 최대값
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":           # 주석 줄 건너뛰기
            while data[pos:pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        end = pos
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b"P5":
        raise ValueError(f"P5 형식의 PGM만 지원합니다: {tokens[0]!r}")
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1                                    # 헤더 뒤 공백 1칸
    dtype = np.uint8 if maxval < 256 else ">u2"
    arr = np.frombuffer(data, dtype=dtype, count=w * h, offset=pos).reshape(h, w)
    return arr.astype(np.float64) * (255.0 / maxval)


class GridMap:
    def __init__(self, yaml_path, robot_radius=0.15, unknown_is_obstacle=True):
        """YAML/PGM 혹은 forward-aligned NPZ를 로드한다. 좌표는 항상 ROS map 기준."""
        path = os.path.expanduser(str(yaml_path))
        self.npz_based = path.lower().endswith(".npz")
        if self.npz_based:
            with np.load(path, allow_pickle=False) as d:
                blocked = np.asarray(d["blocked"])
                if blocked.ndim != 2:
                    raise ValueError("NPZ blocked 배열은 2차원이어야 합니다.")
                self.resolution = float(d["resolution"])
                self.origin_x = float(d["origin_x"])
                self.origin_y = float(d["origin_y"])
                self.origin_yaw = float(d["origin_yaw"])
                self.blocked_raw = (blocked != 0)
                self.free_raw = ~self.blocked_raw
        else:
            with open(path) as f:
                meta = yaml.safe_load(f)
            self.resolution = float(meta["resolution"])
            self.origin_x, self.origin_y = float(meta["origin"][0]), float(meta["origin"][1])
            self.origin_yaw = float(meta["origin"][2]) if len(meta["origin"]) > 2 else 0.0
            negate = int(meta.get("negate", 0))
            occ_th = float(meta.get("occupied_thresh", 0.65))
            free_th = float(meta.get("free_thresh", 0.25))
            img_path = meta["image"]
            if not os.path.isabs(img_path):
                img_path = os.path.join(os.path.dirname(os.path.abspath(path)), img_path)
            img = read_pgm(img_path)
            p_occ = (255.0 - img) / 255.0 if not negate else img / 255.0
            occupied = p_occ > occ_th
            free = (p_occ < free_th) & (img >= 250)
            unknown = ~(free | occupied)
            self.free_raw = np.flipud(free)
            self.blocked_raw = np.flipud(occupied | (unknown if unknown_is_obstacle else False))
        self.height, self.width = self.blocked_raw.shape
        if self.resolution <= 0 or not all(map(math.isfinite, (self.resolution, self.origin_x, self.origin_y, self.origin_yaw))):
            raise ValueError("지도 원점 또는 해상도가 잘못되었습니다.")
        self.robot_radius = robot_radius
        self.blocked = self._inflate(self.blocked_raw, robot_radius)

    # ---------- 좌표 변환 ----------
    def world_to_cell(self, x, y):
        # 초기 전방으로 회전된 NPZ에서 map 월드 좌표를 그리드 로컬 좌표로 변환.
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        col = math.floor((c * dx + s * dy) / self.resolution)
        row = math.floor((-s * dx + c * dy) / self.resolution)
        return row, col

    def cell_to_world(self, row, col):
        u, v = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        x = self.origin_x + c * u - s * v
        y = self.origin_y + s * u + c * v
        return x, y

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row, col):
        return self.in_bounds(row, col) and not self.blocked[row, col]

    # ---------- 장애물 팽창 ----------
    @staticmethod
    def _inflate_with(blocked, radius_cells):
        r = int(math.ceil(radius_cells))
        out = blocked.copy()
        h, w = blocked.shape
        ys, xs = np.nonzero(blocked)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                ny, nx = ys + dy, xs + dx
                ok = (ny >= 0) & (ny < h) & (nx >= 0) & (nx < w)
                out[ny[ok], nx[ok]] = True
        return out

    def _inflate(self, blocked, radius_m):
        return self._inflate_with(blocked, radius_m / self.resolution)

    # ---------- 가까운 빈 칸 찾기 ----------
    def nearest_free(self, row, col, max_radius=15):
        if self.is_free(row, col):
            return row, col
        for r in range(1, max_radius + 1):
            best = None
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    if max(abs(dr), abs(dc)) != r:
                        continue
                    rr, cc = row + dr, col + dc
                    if self.is_free(rr, cc):
                        d = dr * dr + dc * dc
                        if best is None or d < best[0]:
                            best = (d, rr, cc)
            if best:
                return best[1], best[2]
        return None

    # ---------- 직선 시야 확인 ----------
    def line_free(self, a, b):
        (r0, c0), (r1, c1) = a, b
        n = int(max(abs(r1 - r0), abs(c1 - c0))) * 2 + 1
        for i in range(n + 1):
            t = i / n
            r = int(round(r0 + (r1 - r0) * t))
            c = int(round(c0 + (c1 - c0) * t))
            if not self.is_free(r, c):
                return False
        return True


_NEIGHBORS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
]


def _octile(a, b):
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


def astar(grid, start, goal):
    """start, goal: (row, col). 경로 칸 리스트를 반환. 없으면 None."""
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return None
    open_heap = [(0.0, 0.0, start)]
    g_cost = {start: 0.0}
    parent = {}
    closed = set()
    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        for dr, dc, cost in _NEIGHBORS:
            nb = (cur[0] + dr, cur[1] + dc)
            if not grid.is_free(*nb) or nb in closed:
                continue
            # 대각선 이동 시 모서리 끼임 방지: 양옆 두 칸도 비어 있어야 함
            if dr != 0 and dc != 0:
                if not grid.is_free(cur[0] + dr, cur[1]) or not grid.is_free(cur[0], cur[1] + dc):
                    continue
            ng = g + cost
            if ng < g_cost.get(nb, float("inf")):
                g_cost[nb] = ng
                parent[nb] = cur
                heapq.heappush(open_heap, (ng + _octile(nb, goal), ng, nb))
    return None


def simplify(grid, cells):
    """일직선으로 갈 수 있으면 중간 칸을 건너뛰어 꼭짓점만 남긴다."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not grid.line_free(cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


def plan(grid, start_xy, goal_xy, simplify_path=True):
    """월드 좌표(m)로 시작/목표를 받아 월드 좌표 경로 [(x, y), ...]를 반환. 실패 시 None."""
    # 시작 셀은 TF(map -> base_footprint)의 현재 차량 중심 좌표가 속한 셀 그대로 사용.
    # 주변 빈 셀로 이동시키면 실제 위치와 A* 시작점이 달라질 수 있으므로 보정하지 않는다.
    s = grid.world_to_cell(*start_xy)
    g = grid.world_to_cell(*goal_xy)
    if not grid.is_free(*s) or not grid.is_free(*g):
        # 위치 오차/장애물 팽창으로 현재 셀이 막혔다면 실패를 명확히 알린다.
        return None
    if grid.npz_based:
        # 사용자가 구현한 기존 4방향 A*(x,y): NPZ blocked[y,x]를 그대로 전달.
        cells_xy = astar_4way(grid.blocked.astype(np.uint8), grid.width, grid.height,
                              (s[1], s[0]), (g[1], g[0]))
        cells = [(y, x) for x, y in cells_xy]
    else:
        cells = astar(grid, s, g)
    if not cells:
        return None
    if simplify_path:
        cells = simplify(grid, cells)
    pts = [grid.cell_to_world(r, c) for r, c in cells]
    # 첫/끝점은 실제 요청 좌표로 맞춰서 목표 도달 판정이 정확하도록 함
    if grid.is_free(*grid.world_to_cell(*goal_xy)):
        pts[-1] = goal_xy
    return pts


if __name__ == "__main__":
    import sys
    yaml_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/maps/room_map.yaml")
    gm = GridMap(yaml_path)
    print(f"맵 {gm.width}x{gm.height} 칸, 해상도 {gm.resolution} m, 원점 ({gm.origin_x}, {gm.origin_y})")
    print(f"막힌 칸(팽창 전/후): {int(gm.blocked_raw.sum())} / {int(gm.blocked.sum())}")
