"""검증: 실시간 로봇 위치가 포함된 NPZ 셀만 A* 시작점으로 사용한다."""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import numpy as np
import astar_planner
from astar_planner import GridMap, plan
from amr_server import Core, Conflict

for theta in (0.0, math.pi / 2, -0.83):
    with TemporaryDirectory() as tmp:
        path = Path(tmp) / 'test.npz'
        raw = np.zeros((19, 21), dtype=np.uint8)
        # 특정 위치는 통행 불가로 설정. 바로 이웃은 통행 가능.
        raw[7, 8] = 1
        np.savez_compressed(path, blocked=raw, occupancy=np.where(raw == 0, 0, 100).astype(np.int8),
                            resolution=0.05, origin_x=-0.7, origin_y=0.31, origin_yaw=theta)
        grid = GridMap(str(path), robot_radius=0)
        intended_start = (5, 4)  # row, col
        intended_goal = (13, 16)
        robot_xy = grid.cell_to_world(*intended_start)
        goal_xy = grid.cell_to_world(*intended_goal)
        assert grid.world_to_cell(*robot_xy) == intended_start
        called = []
        original = astar_planner.astar_4way
        def spy(g, w, h, start, goal):
            called.append((start, goal))
            return original(g, w, h, start, goal)
        try:
            astar_planner.astar_4way = spy
            route = plan(grid, robot_xy, goal_xy, simplify_path=False)
            assert route is not None and len(route) > 1
            assert called == [((intended_start[1], intended_start[0]),
                               (intended_goal[1], intended_goal[0]))], called
            assert grid.world_to_cell(*route[0]) == intended_start
        finally:
            astar_planner.astar_4way = original

        stub = SimpleNamespace(mode='nav', plan_grid=grid,
                               ros=SimpleNamespace(pose=[*robot_xy, 0]))
        preview = Core.preview_goal(stub, goal_xy)
        assert preview['start_cell'] == [intended_start[1], intended_start[0]]
        assert preview['goal_cell'] == [intended_goal[1], intended_goal[0]]
        assert preview['path']

        blocked_xy = grid.cell_to_world(7, 8)
        assert grid.nearest_free(7, 8) is not None, 'This test needs a nearby free cell'
        assert plan(grid, blocked_xy, goal_xy) is None, 'Must not silently move start cell'
        stub.ros.pose = [*blocked_xy, 0]
        try:
            Core.preview_goal(stub, goal_xy)
        except Conflict as exc:
            assert '시작 셀(row=7, col=8)' in str(exc)
        else:
            raise AssertionError('Blocked robot start was silently replaced')

print('Exact TF robot cell -> NPZ row/col -> A* start: PASS (3 rotations)')
print('Blocked actual start rejects path instead of nearest-free substitution: PASS')
