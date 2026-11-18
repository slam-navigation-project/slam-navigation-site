#!/usr/bin/env python3
"""
경로 추종 + 웨이포인트 왕복 로직 (ROS 없이 단독 동작, 시뮬레이션으로 검증 가능)

차는 차동구동(좌우 바퀴)이라 (v, w) = (전진속도, 회전속도)만 낼 수 있다.
- 목표 꼭짓점과 차 방향 차이가 크면 제자리 회전, 작으면 전진하면서 조향한다.
"""
import math


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathFollower:
    def __init__(self, linear_speed=0.08, turn_speed=0.5, rotate_threshold=0.5,
                 k_ang=1.5, vertex_tol=0.10, goal_tol=0.08):
        self.linear_speed = linear_speed
        self.turn_speed = turn_speed          # 제자리 회전 / 조향 최대 각속도 (rad/s)
        self.rotate_threshold = rotate_threshold  # 이 각도(rad)보다 틀어져 있으면 제자리 회전
        self.k_ang = k_ang
        self.vertex_tol = vertex_tol
        self.goal_tol = goal_tol
        self.path = []
        self.idx = 0

    def set_path(self, path):
        """path: [(x, y), ...] 첫 점은 현재 위치 근처이므로 건너뛰고 두 번째 점부터 향한다."""
        self.path = list(path)
        self.idx = 1 if len(self.path) > 1 else 0

    @property
    def done(self):
        return not self.path or self.idx >= len(self.path)

    def step(self, x, y, yaw):
        """현재 pose로 (v, w)를 계산. 경로를 다 지나면 (0, 0, True)."""
        # 지나간 꼭짓점은 건너뛴다
        while self.idx < len(self.path):
            tx, ty = self.path[self.idx]
            last = self.idx == len(self.path) - 1
            tol = self.goal_tol if last else self.vertex_tol
            if math.hypot(tx - x, ty - y) <= tol:
                self.idx += 1
            else:
                break
        if self.idx >= len(self.path):
            return 0.0, 0.0, True

        tx, ty = self.path[self.idx]
        err = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > self.rotate_threshold:
            w = self.turn_speed if err > 0 else -self.turn_speed
            return 0.0, w, False
        w = max(-self.turn_speed, min(self.turn_speed, self.k_ang * err))
        return self.linear_speed, w, False


class PingPong:
    """웨이포인트 [w0, w1, ..., wn]을 w0→wn→w0→... 순서로 계속 왕복한다."""

    def __init__(self):
        self.points = []
        self.i = 0          # 지금 향하는 웨이포인트 번호
        self.dir = 1

    def clear(self):
        self.points = []
        self.i = 0
        self.dir = 1

    def add(self, p):
        self.points.append(p)

    def current(self):
        if not self.points:
            return None
        return self.points[self.i]

    def advance(self):
        """다음 목표로 넘어간다. 끝점이면 방향을 뒤집는다."""
        n = len(self.points)
        if n <= 1:
            return
        nxt = self.i + self.dir
        if nxt >= n or nxt < 0:
            self.dir = -self.dir
            nxt = self.i + self.dir
        self.i = nxt
