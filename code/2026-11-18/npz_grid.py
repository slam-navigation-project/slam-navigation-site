"""SLAM OccupancyGrid → 초기 AMR 전방(+Y) 정렬 NPZ 및 웹 지도 로딩.

배열 인덱스는 grid[y, x], y=0은 지도 로컬 좌표계의 아래쪽.
origin_x/y/yaw와 resolution을 함께 저장하여 map 월드 좌표를 복구한다.
"""
import math
import os
import shutil
from pathlib import Path

import numpy as np

MAP_EXTS = ("npz", "pgm", "yaml", "posegraph", "data")


def save_aligned_npz(path, map_snapshot, initial_pose):
    """map_snapshot은 amr_server.RosBridge.latest_map, initial_pose는 첫 map 기준 자세."""
    if initial_pose is None:
        raise ValueError("Mapping 초기 로봇 방향이 없습니다. Mapping을 새로 시작해 주세요.")
    height, width = int(map_snapshot["h"]), int(map_snapshot["w"])
    res = float(map_snapshot["res"])
    raw = np.asarray(map_snapshot["data"], dtype=np.int8).reshape(height, width)
    old_x, old_y = float(map_snapshot["ox"]), float(map_snapshot["oy"])
    old_yaw = float(map_snapshot.get("yaw", 0.0))
    rx, ry, robot_yaw = map(float, initial_pose)
    if not all(map(math.isfinite, (res, old_x, old_y, old_yaw, rx, ry, robot_yaw))) or res <= 0:
        raise ValueError("지도 해상도 또는 좌표가 올바르지 않습니다.")

    new_yaw = robot_yaw - math.pi / 2.0  # 로봇 첫 전방(map yaw)이 새 +Y
    c, s = math.cos(new_yaw), math.sin(new_yaw)
    co, so = math.cos(old_yaw), math.sin(old_yaw)

    bx = np.array([0., width * res, 0., width * res])
    by = np.array([0., 0., height * res, height * res])
    dx = old_x + co * bx - so * by - rx
    dy = old_y + so * bx + co * by - ry
    u, v = c * dx + s * dy, -s * dx + c * dy
    umin = math.floor(u.min() / res + 1e-9) * res
    vmin = math.floor(v.min() / res + 1e-9) * res
    umax = math.ceil(u.max() / res - 1e-9) * res
    vmax = math.ceil(v.max() / res - 1e-9) * res
    new_w = max(1, round((umax - umin) / res))
    new_h = max(1, round((vmax - vmin) / res))
    if new_w * new_h > 20_000_000:
        raise ValueError("회전된 지도가 너무 큽니다.")

    occupancy = np.full((new_h, new_w), -1, dtype=np.int8)
    ucenters = umin + (np.arange(new_w) + 0.5) * res
    for row in range(new_h):
        vcenter = vmin + (row + 0.5) * res
        ddx = c * ucenters - s * vcenter + rx - old_x
        ddy = s * ucenters + c * vcenter + ry - old_y
        col0 = np.floor((co * ddx + so * ddy) / res + 1e-9).astype(np.int64)
        row0 = np.floor((-so * ddx + co * ddy) / res + 1e-9).astype(np.int64)
        inside = (col0 >= 0) & (col0 < width) & (row0 >= 0) & (row0 < height)
        occupancy[row, inside] = raw[row0[inside], col0[inside]]

    blocked = ((occupancy == -1) | (occupancy >= 50)).astype(np.uint8)
    origin_x = rx + c * umin - s * vmin
    origin_y = ry + s * umin + c * vmin
    with open(path, "wb") as f:
        np.savez_compressed(
            f, occupancy=occupancy, blocked=blocked, resolution=np.float64(res),
            width=np.int32(new_w), height=np.int32(new_h),
            origin_x=np.float64(origin_x), origin_y=np.float64(origin_y),
            origin_yaw=np.float64(new_yaw),
            frame_id=np.str_(map_snapshot.get("frame_id", "map")),
            aligned_to_initial_forward=np.bool_(True),
            initial_robot_x=np.float64(rx), initial_robot_y=np.float64(ry),
            initial_robot_yaw=np.float64(robot_yaw),
        )
    return new_w, new_h


def load_saved_npz(path):
    """NPZ의 occupancy로 웹 표시용 F/O/U 행(위→아래)과 지도 메타데이터를 구성."""
    with np.load(path, allow_pickle=False) as data:
        raw = np.asarray(data["occupancy"], dtype=np.int8)
        blocked = np.asarray(data["blocked"])
        if raw.ndim != 2 or raw.shape != blocked.shape:
            raise ValueError("NPZ occupancy/blocked의 배열 크기가 다릅니다.")
        h, w = raw.shape
        res = float(data["resolution"])
        ox, oy, yaw = (float(data[key]) for key in ("origin_x", "origin_y", "origin_yaw"))
        aligned = bool(data["aligned_to_initial_forward"]) if "aligned_to_initial_forward" in data else False
    if not all(map(math.isfinite, (res, ox, oy, yaw))) or res <= 0:
        raise ValueError("NPZ 원점과 해상도 오류")
    # 화면의 맨 위부터 행을 반환함. 50 미만이지만 미탐색(-1)은 'U'.
    rows = []
    for row in raw[::-1]:
        rows.append("".join("U" if v < 0 else ("O" if v >= 50 else "F") for v in row))
    return {"w": w, "h": h, "res": res, "ox": ox, "oy": oy,
            "yaw": yaw, "rows": rows, "aligned": aligned}


def install_map_bundle(temp_base, final_base):
    """동일 파일시스템 내 임시 파일 5개가 모두 준비된 뒤 설치. 오류 시 백업 복원."""
    src, dst = Path(temp_base), Path(final_base)
    for ext in MAP_EXTS:
        p = Path(f"{src}.{ext}")
        if not p.is_file() or p.stat().st_size == 0:
            raise FileNotFoundError(f"저장 실패 또는 빈 파일: {p}")
    backup = src.parent / ".previous"
    backup.mkdir(mode=0o700, exist_ok=False)
    old, new = [], []
    try:
        for ext in MAP_EXTS:
            d = Path(f"{dst}.{ext}")
            if d.exists():
                os.replace(d, backup / d.name)
                old.append(ext)
        for ext in MAP_EXTS:
            os.replace(Path(f"{src}.{ext}"), Path(f"{dst}.{ext}"))
            new.append(ext)
    except Exception:
        for ext in new:
            Path(f"{dst}.{ext}").unlink(missing_ok=True)
        for ext in old:
            os.replace(backup / Path(f"{dst}.{ext}").name, Path(f"{dst}.{ext}"))
        raise
    else:
        shutil.rmtree(backup)
