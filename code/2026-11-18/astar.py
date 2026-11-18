"""NPZ blocked[y, x] 지도에서 4방향 A* 최단경로를 탐색한다."""
import heapq
import math
import numpy as np


def get_Euclidean(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


def get_Diagonal(x1, y1, x2, y2):
    dx, dy = abs(x1 - x2), abs(y1 - y2)
    return min(dx, dy) * math.sqrt(2) + abs(dx - dy)


def get_Manhattan(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)


def astar(grid_map, W_map, H_map, start, goal):
    """start/goal: (x,y); 반환: [(x,y), ...], 경로가 없으면 [].

    상하좌우 4방향, 이동비용 1, 휴리스틱 Manhattan.
    빈 공간(blocked == 0)만 탐색하며, 경로 길이는 항상 최단 셀 이동 횟수이다.
    """
    grid = np.asarray(grid_map)
    if grid.ndim != 2 or grid.shape != (H_map, W_map):
        raise ValueError(f"지도 크기가 맞지 않습니다: shape={grid.shape}, W={W_map}, H={H_map}")

    sx, sy = map(int, start)
    gx, gy = map(int, goal)
    for point, name in (((sx, sy), "시작"), ((gx, gy), "도착")):
        x, y = point
        if not (0 <= x < W_map and 0 <= y < H_map):
            raise ValueError(f"{name} 좌표 {point}가 지도 범위를 벗어났습니다.")
        if grid[y, x] != 0:
            raise ValueError(f"{name} 좌표 {point}는 통행 불가 셀입니다.")

    if (sx, sy) == (gx, gy):
        return [(sx, sy)]

    start_pt = (sx, sy)
    goal_pt = (gx, gy)
    g_score = {start_pt: 0}
    parent = {}
    closed = set()
    # (F, G, x, y). G is included to distinguish stale heap entries.
    heap = [(get_Manhattan(sx, sy, gx, gy), 0, sx, sy)]
    directions = ((0, 1), (1, 0), (0, -1), (-1, 0))

    while heap:
        _, current_g, x, y = heapq.heappop(heap)
        current = (x, y)
        if current in closed or current_g != g_score.get(current):
            continue

        if current == goal_pt:
            path = [goal_pt]
            while path[-1] != start_pt:
                path.append(parent[path[-1]])
            path.reverse()
            return path

        closed.add(current)
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            nxt = (nx, ny)
            if not (0 <= nx < W_map and 0 <= ny < H_map):
                continue
            if grid[ny, nx] != 0 or nxt in closed:
                continue
            new_g = current_g + 1
            if new_g < g_score.get(nxt, float("inf")):
                g_score[nxt] = new_g
                parent[nxt] = current
                h = get_Manhattan(nx, ny, gx, gy)
                heapq.heappush(heap, (new_g + h, new_g, nx, ny))

    return []
