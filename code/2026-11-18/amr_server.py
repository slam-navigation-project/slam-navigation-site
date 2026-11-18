#!/usr/bin/env python3
"""
AMR 웹 서버 (라즈베리파이에서 실행)

브라우저(PC/휴대폰)에서 열어 쓰는 화면(index.html)을 서빙하고, 아래 일을 한다.
  1. [맵 생성 시작]  SLAM 런치를 껐다가 새로 켠다 (맵이 비어 있는 상태로 시작)
  2. 만들어지는 /map 을 화면에 계속 전달, 수동 조종(/cmd_vel) 명령을 받아 전달
  3. [생성 완료]    slam_toolbox 로 맵을 저장(~/maps/이름.pgm/.yaml)하고, 주행 노드(waypoint_nav_node.py)를 띄워 웨이포인트 모드로 전환
  4. 웨이포인트 명령(go/stop/clear)은 주행 노드와 /tmp 파일로 주고받는다 (GUI 때와 같은 방식)

실행 (ROS가 source 된 터미널에서. SLAM은 이 서버가 알아서 켠다):
  source /opt/ros/humble/setup.bash
  python3 amr_server.py                       # Tailscale IP의 8080 포트로 열림
  python3 amr_server.py --host 100.x.y.z --port 8080 --launch ~/ros2_amr_ws/src/riding_test/slam_wheel_odom.launch.py

같은 폴더에 index.html, waypoint_nav_node.py, astar_planner.py, path_follower.py, map_geometry.py 가 있어야 한다.
브라우저에서  http://<파이 Tailscale IP>:8080  으로 접속.

보안: 이 주소에 접속할 수 있으면 차를 움직일 수 있다. 기본값은 Tailscale 주소에만 열고, POST 는 application/json 만 받는다.
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import tempfile
import shutil
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from map_geometry import CMD_FILE, STATE_FILE, make_command, parse_map_yaml, parse_state   # noqa: E402
from astar_planner import GridMap, plan   # noqa: E402
from npz_grid import save_aligned_npz, load_saved_npz, install_map_bundle   # noqa: E402

MAX_LINEAR = 0.20      # 수동 조종 상한 (m/s)
MAX_ANGULAR = 1.0      # (rad/s)
DRIVE_TIMEOUT = 0.4    # 이 시간 동안 새 조종 명령이 없으면 자동 정지 (s)


# ======================================================================
# 순수 함수 (ROS 없이 테스트 가능)
# ======================================================================
def occupancy_to_rows(width, height, data, occ_th=65, free_th=25):
    """nav_msgs/OccupancyGrid 데이터를 화면용 문자열 행(위=북쪽 -> 아래)으로 바꾼다. F=빈 공간, O=장애물, U=미탐색.
    OccupancyGrid 는 row 0 이 남쪽(원점 쪽)이라 뒤집어서 만든다."""
    rows = []
    for r in range(height - 1, -1, -1):
        base = r * width
        chars = []
        for c in range(width):
            v = data[base + c]
            if v < 0:
                chars.append("U")
            elif v >= occ_th:
                chars.append("O")
            elif v < free_th:
                chars.append("F")
            else:
                chars.append("U")
        rows.append("".join(chars))
    return rows


def read_pgm_pixels(path):
    """P5 PGM 을 (너비, 높이, 픽셀 바이트)로 읽는다 (numpy 없이)."""
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        end = pos
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b"P5":
        raise ValueError("P5 형식의 PGM만 지원합니다")
    w, h = int(tokens[1]), int(tokens[2])
    pos += 1
    return w, h, data[pos:pos + w * h]


def load_saved_map(yaml_path):
    """저장된 맵(yaml+pgm)을 화면용 dict 로 읽는다. 흰색(>=250)만 빈 공간으로 본다 (A* 쪽과 같은 규칙)."""
    yaml_path = Path(yaml_path)
    meta = parse_map_yaml(yaml_path.read_text(encoding="utf-8"))
    pgm = Path(meta["image"])
    if not pgm.is_absolute():
        pgm = yaml_path.parent / pgm
    w, h, pix = read_pgm_pixels(pgm)
    rows = []
    for r in range(h):
        line = pix[r * w:(r + 1) * w]
        rows.append("".join("F" if v >= 250 else ("O" if v <= 50 else "U") for v in line))
    return {"w": w, "h": h, "res": meta["resolution"], "ox": meta["origin"][0], "oy": meta["origin"][1], "rows": rows}


def clean_name(name):
    name = re.sub(r"[^A-Za-z0-9_-]", "_", (name or "").strip())[:40]
    return name or time.strftime("map_%Y%m%d_%H%M%S")


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ======================================================================
# 프로세스 관리 (SLAM 런치, 주행 노드)
# ======================================================================
class Proc:
    def __init__(self, name, cmd, log_path):
        self.name, self.cmd, self.log_path = name, cmd, log_path
        self.p = None

    def start(self):
        log = open(self.log_path, "w")
        self.p = subprocess.Popen(self.cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self):
        return self.p is not None and self.p.poll() is None

    def stop(self, timeout=8.0):
        """프로세스 그룹에 SIGINT(Ctrl+C 와 같음) -> 안 꺼지면 TERM -> KILL."""
        if self.p is None:
            return
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            if self.p.poll() is not None:
                break
            try:
                os.killpg(self.p.pid, sig)
                self.p.wait(timeout=wait)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        self.p = None


# ======================================================================
# ROS 연결 (/map 구독, TF 위치, /cmd_vel 발행, slam_toolbox 서비스)
# ======================================================================
class RosBridge:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        from slam_toolbox.srv import SaveMap, SerializePoseGraph

        self.rclpy, self.Twist, self.SaveMap, self.SerializePoseGraph = rclpy, Twist, SaveMap, SerializePoseGraph
        rclpy.init()
        self.node = Node("amr_web_server")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.save_cli = self.node.create_client(SaveMap, "/slam_toolbox/save_map")
        self.ser_cli = self.node.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

        self.latest_map = None
        self.initial_pose = None     # 첫 유효 map->base_footprint, Mapping 초기 방향
        self.capture_initial = False
        self.map_seq = 0
        self.pose = None
        self.latest_scan = None
        self._laser_tf = None
        self.drive_cmd = (0.0, 0.0, 0.0)     # v, w, 받은 시각
        self.was_driving = False
        self.node.create_timer(0.1, self._tick)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _on_map(self, msg):
        self.map_seq += 1
        q = msg.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.latest_map = {"stamp": str(self.map_seq), "w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
                           "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
                           "yaw": yaw, "frame_id": msg.header.frame_id, "data": list(msg.data)}

    def _laser_offset(self, frame_id):
        # base_footprint -> 라이다 프레임 (장착 위치와 방향). launch 파일의 static TF 값을 그대로 읽는다. 한 번 성공하면 저장해서 재사용
        if self._laser_tf is not None and self._laser_tf[0] == frame_id:
            return self._laser_tf[1]
        try:
            t = self.tf_buffer.lookup_transform("base_footprint", frame_id, self.rclpy.time.Time())
        except Exception:
            return None                                    # 아직 TF 가 없으면 이번 스캔은 건너뛴다 (정렬 안 된 채로 그리지 않음)
        q, tr = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._laser_tf = (frame_id, (yaw, tr.x, tr.y))
        return self._laser_tf[1]

    def _on_scan(self, msg):
        # 화면용으로 360개 안팎으로 줄이고, 유효하지 않은 값(inf/0/범위 밖)은 None 으로 둔다
        n = len(msg.ranges)
        if n == 0:
            return
        off = self._laser_offset(msg.header.frame_id)
        if off is None:
            return
        yaw, lx, ly = off
        step = max(1, n // 360)
        out = []
        for x in msg.ranges[::step]:
            out.append(round(float(x), 2) if math.isfinite(x) and msg.range_min <= x <= msg.range_max else None)
        # a0 는 '차 기준 각도'(앞 = 0, 왼쪽 +): 라이다가 뒤를 보고 달려 있으면 yaw = π 만큼 돌려서 보낸다
        self.latest_scan = {"a0": msg.angle_min + yaw, "da": msg.angle_increment * step, "lx": lx, "ly": ly,
                            "rmax": float(msg.range_max) if math.isfinite(msg.range_max) else 12.0, "r": out, "ts": time.time()}

    def clear_map(self):
        self.latest_map = None
        self.pose = None
        self.initial_pose = None
        self.capture_initial = True

    def _tick(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", self.rclpy.time.Time())
            q, tr = t.transform.rotation, t.transform.translation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = [tr.x, tr.y, yaw]
            # /map 수신 후 첫 번째 map 기준 자세를 기록. 저장 당시 자세로 대체하지 않음.
            if self.capture_initial and self.latest_map is not None:
                self.initial_pose = tuple(self.pose)
                self.capture_initial = False
                self.node.get_logger().info(f"초기 전방 캡처 yaw={yaw:.3f}rad")
        except Exception:
            self.pose = None
        v, w, stamp = self.drive_cmd
        fresh = time.time() - stamp < DRIVE_TIMEOUT and (v != 0.0 or w != 0.0)
        if fresh:
            msg = self.Twist(); msg.linear.x, msg.angular.z = v, w
            self.cmd_pub.publish(msg)
            self.was_driving = True
        elif self.was_driving:                   # 조종이 끝난 직후에만 0 속도를 보낸다 (평소엔 다른 노드의 /cmd_vel 과 안 겹치게 발행하지 않음)
            self.cmd_pub.publish(self.Twist())
            self.was_driving = False

    def set_drive(self, v, w):
        self.drive_cmd = (v, w, time.time())

    def _call(self, client, req, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def save_map(self, base_path, timeout=20.0):
        req = self.SaveMap.Request()
        req.name.data = base_path
        return self._call(self.save_cli, req, timeout) is not None

    def serialize(self, base_path, timeout=20.0):
        req = self.SerializePoseGraph.Request()
        req.filename = base_path
        return self._call(self.ser_cli, req, timeout) is not None


# ======================================================================
# 핵심 로직: 모드(idle / mapping / nav)와 전환
# ======================================================================
class Conflict(Exception):
    pass


class Core:
    def __init__(self, ros, launch_file, maps_dir, nav_node, proc_factory=Proc):
        self.ros = ros
        self.launch_file = str(Path(launch_file).expanduser())
        self.maps_dir = Path(maps_dir).expanduser()
        self.nav_node = str(nav_node)
        self.proc_factory = proc_factory
        self.mode = "idle"
        self.msg = "맵 생성을 시작하세요."
        self.slam = None
        self.nav = None
        self.frozen = None            # 저장된 NPZ 맵 (화면용 dict)
        self.saved_yaml = None
        self.saved_npz = None
        self.plan_grid = None
        self.nav_radius = 0.12
        self._manual_until = 0.0      # 마지막 수동 조종 명령 + 1초 (이 시간이 지났다가 다시 움직이면 새 조종 시작으로 본다)
        self.trans = threading.Lock() # 모드 전환은 한 번에 하나만
        self._cache = (None, None)    # (stamp, 변환된 dict)

    # ---------- 전환 ----------
    def _locked(self):
        if not self.trans.acquire(blocking=False):
            raise Conflict("다른 작업을 처리 중이에요. 잠시 후 다시 눌러주세요.")

    def start_mapping(self):
        self._locked()
        try:
            self.mode = "starting"
            self.msg = "기존 프로세스를 정리하고 SLAM을 새로 켜는 중..."
            self._stop_nav()
            self._stop_slam()
            self.ros.clear_map()
            self.frozen, self.saved_yaml, self.saved_npz, self.plan_grid = None, None, None, None
            self.slam = self.proc_factory("slam", ["ros2", "launch", self.launch_file], "/tmp/amr_slam.log")
            self.slam.start()
            time.sleep(1.0)
            if not self.slam.alive():
                self.mode = "idle"
                self.msg = "SLAM 실행 실패 (로그: /tmp/amr_slam.log)"
                raise Conflict(self.msg)
            self.mode = "mapping"
            self.msg = "맵 만드는 중: 수동으로 돌아다니며 방을 훑으세요."
        finally:
            self.trans.release()

    def finish_mapping(self, name=None, radius=0.12, speed=0.08):
        """PGM/YAML + NPZ(초기 전방 정렬) + SLAM 직렬화 5개 저장 후 단일 목적지 모드."""
        self._locked()
        temp_dir = None
        try:
            if self.mode != "mapping":
                raise Conflict("맵 생성 중이 아니에요.")
            snapshot = self.ros.latest_map
            initial_pose = self.ros.initial_pose
            if snapshot is None or initial_pose is None:
                raise Conflict("지도 또는 Mapping 초기 로봇 방향이 없습니다. SLAM/TF를 확인하고 다시 시작하세요.")
            self.ros.set_drive(0.0, 0.0)
            self._manual_until = 0.0
            name = clean_name(name)
            self.maps_dir.mkdir(parents=True, exist_ok=True)
            final_base = str(self.maps_dir / name)
            temp_dir = tempfile.mkdtemp(prefix=".map_save_", dir=self.maps_dir)
            base = str(Path(temp_dir) / name)
            self.mode = "saving"
            self.msg = f"지도 5개 파일 저장 중: {name}"
            try:
                if not self.ros.save_map(base, timeout=45.0):
                    raise Conflict("SLAM Toolbox PGM/YAML 저장 실패")
                if not self.ros.serialize(base, timeout=75.0):
                    raise Conflict("SLAM Toolbox DATA/PoseGraph 저장 실패")
                save_aligned_npz(base + ".npz", snapshot, initial_pose)
                # 모두 준비된 뒤 5개를 한꺼번에 기존 이름으로 교체
                install_map_bundle(base, final_base)
                self.saved_yaml = final_base + ".yaml"
                self.saved_npz = final_base + ".npz"
                self.plan_grid = GridMap(self.saved_npz, robot_radius=float(radius))
                self.nav_radius = float(radius)
                self.frozen = load_saved_npz(self.saved_npz)
                self.frozen["stamp"] = "saved-" + name + "-" + str(time.time_ns())
                self._start_nav(radius, speed)
            except Exception as exc:
                self.mode = "mapping"
                self.frozen, self.saved_yaml, self.saved_npz, self.plan_grid = None, None, None, None
                self.msg = str(exc) or "맵 저장 중 오류"
                raise Conflict(self.msg) from exc
            self.mode = "nav"
            self.msg = f"맵 {name} 저장 완료. 흰색 칸을 한 번 클릭하면 A* 경로를 미리 볼 수 있어요."
            return {"name": name, "yaml": self.saved_yaml, "npz": self.saved_npz}
        finally:
            if temp_dir and Path(temp_dir).exists():
                # 백업 복원이 실패한 경우는 안전을 위해 남겨둔다.
                if (Path(temp_dir) / '.previous').exists():
                    self.msg = "지도 백업 복원 확인 필요: " + temp_dir
                else:
                    shutil.rmtree(temp_dir, ignore_errors=True)
            self.trans.release()

    def restart_nav(self, radius, speed):
        self._locked()
        try:
            if self.mode != "nav":
                raise Conflict("주행 모드가 아니에요.")
            self._stop_nav()
            if self.saved_npz:
                self.plan_grid = GridMap(self.saved_npz, robot_radius=float(radius))
                self.nav_radius = float(radius)
            self._start_nav(radius, speed)
            self.msg = f"주행 설정 적용: 차 반경 {radius} m, 속도 {speed} m/s"
        finally:
            self.trans.release()

    def stop_all(self):
        self._locked()
        try:
            self.ros.set_drive(0.0, 0.0)
            self._stop_nav()
            self._stop_slam()
            self.mode = "idle"
            self.msg = "SLAM을 껐어요."
        finally:
            self.trans.release()

    def _start_nav(self, radius, speed):
        radius = clamp(float(radius), 0.03, 0.5)
        speed = clamp(float(speed), 0.02, 0.2)
        for f in (STATE_FILE,):
            try:
                os.remove(f)
            except OSError:
                pass
        self.nav = self.proc_factory("nav", [sys.executable, self.nav_node, "--ros-args", "-p", f"map_yaml:={self.saved_yaml}",
                                             "-p", f"map_npz:={self.saved_npz}", "-p", f"robot_radius:={radius}", "-p", f"linear_speed:={speed}"], "/tmp/amr_nav.log")
        self.nav.start()

    def _stop_nav(self):
        if self.nav is not None:
            self.nav.stop()
            self.nav = None

    def _stop_slam(self):
        if self.slam is not None:
            self.slam.stop()
            self.slam = None

    # ---------- 입력 ----------
    def drive(self, v, w):
        v, w = clamp(float(v), -MAX_LINEAR, MAX_LINEAR), clamp(float(w), -MAX_ANGULAR, MAX_ANGULAR)
        moving = v != 0.0 or w != 0.0
        if not moving:                            # 정지 명령은 어느 모드에서든 항상 받는다
            self._manual_until = 0.0
            self.ros.set_drive(0.0, 0.0)
            return
        if self.mode not in ("mapping", "nav"):
            raise Conflict("SLAM이 켜져 있지 않아요.")
        now = time.time()
        if self.mode == "nav" and now > self._manual_until:
            self._write_cmd("stop")               # 새 수동 조종이 시작될 때 자동 주행을 먼저 멈춰서 /cmd_vel 이 겹치지 않게 한다
        self._manual_until = now + 1.0
        self.ros.set_drive(v, w)

    def send_cmd(self, cmd, points=None):
        if self.mode != "nav":
            raise Conflict("주행 모드가 아니에요. 맵 생성을 먼저 끝내세요.")
        if cmd not in ("go", "stop", "clear"):
            raise Conflict("알 수 없는 명령이에요.")
        if self.nav is None or not self.nav.alive():
            raise Conflict("주행 노드가 꺼져 있어요 (로그: /tmp/amr_nav.log).")
        if cmd == "go":
            if not isinstance(points, list) or len(points) != 1:
                raise Conflict("도착지는 하나만 지정할 수 있습니다.")
            try:
                goal = [float(points[0][0]), float(points[0][1])]
            except (IndexError, TypeError, ValueError) as exc:
                raise Conflict("도착지 좌표가 올바르지 않습니다.") from exc
            self.preview_goal(goal)  # 이동 불가능한 셀/경로 없음/현재 위치 없음이면 주행 거부
            self.ros.set_drive(0.0, 0.0)
            self._manual_until = 0.0
            points = [goal]
        self._write_cmd(cmd, points)

    def preview_goal(self, goal):
        """목적지 하나만 클릭 → 현재 map TF 위치에서 A* 계산. 구동 명령은 보내지 않는다."""
        if self.mode != "nav" or self.plan_grid is None:
            raise Conflict("맵 생성 완료 후 목적지를 지정하세요.")
        if self.ros.pose is None:
            raise Conflict("현재 차량 위치(map→base_footprint TF)를 수신하지 못했습니다.")
        if not isinstance(goal, (tuple, list)) or len(goal) != 2:
            raise Conflict("도착지 좌표 2개가 필요합니다.")
        try:
            x, y = map(float, goal)
        except (ValueError, TypeError) as exc:
            raise Conflict("도착지 좌표가 숫자가 아닙니다.") from exc
        if not (math.isfinite(x) and math.isfinite(y)):
            raise Conflict("도착지 좌표는 유한한 숫자여야 합니다.")
        g = self.plan_grid
        # 근처 빈 칸으로 보정하지 않음: 클릭한 셀 자체가 이동 가능해야 한다.
        rc = g.world_to_cell(x, y)
        if not g.is_free(*rc):
            raise Conflict("클릭한 목적지가 장애물/미탐색/안전거리 영역입니다.")
        robot_xy = tuple(self.ros.pose[:2])  # 실시간 map -> base_footprint TF (미터)
        # TF의 실제 차량 중심이 포함된 셀을 그대로 사용. 주변 셀로 임의 이동 금지.
        start_rc = g.world_to_cell(*robot_xy)
        if not g.in_bounds(*start_rc):
            raise Conflict(f"현재 차량의 시작 셀(row={start_rc[0]}, col={start_rc[1]})이 지도 밖입니다.")
        if not g.is_free(*start_rc):
            raise Conflict(
                f"현재 차량의 시작 셀(row={start_rc[0]}, col={start_rc[1]})이 통행 불가입니다. "
                "TF 위치, NPZ 원점/회전각, 로봇 반경(장애물 팽창)을 확인하세요."
            )
        route = plan(g, robot_xy, (x, y), simplify_path=False)
        if not route:
            raise Conflict("현재 차량 위치에서 목적지까지 A* 경로를 찾지 못했습니다.")
        return {"ok": True, "start": list(robot_xy), "goal": [x, y],
                "start_cell": list(reversed(start_rc)), "goal_cell": list(reversed(rc)),
                "path": [[float(px), float(py)] for px, py in route]}

    def _write_cmd(self, cmd, points=None):
        tmp = CMD_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write(make_command(cmd, points))
        os.replace(tmp, CMD_FILE)

    # ---------- 출력 ----------
    def state(self):
        st = {"mode": self.mode, "msg": self.msg, "pose": self.ros.pose, "active": False, "target_index": None, "path": [],
              "map_stamp": self._map_stamp(), "saved": self.saved_yaml, "time": time.time()}
        if self.mode == "nav":
            if self.nav is not None and not self.nav.alive():
                st["msg"] = "주행 노드가 꺼졌어요 (로그: /tmp/amr_nav.log)"
            try:
                with open(STATE_FILE) as f:
                    ns = parse_state(f.read())
            except OSError:
                ns = None
            if ns and time.time() - ns.get("time", 0) < 3.0:
                st.update(active=bool(ns.get("active")), target_index=ns.get("target_index"), path=ns.get("path") or [])
                st["pose"] = ns.get("pose") or st["pose"]
                if ns.get("msg"):
                    st["msg"] = ns["msg"]
        return st

    def _map_stamp(self):
        if self.mode == "nav" and self.frozen:
            return self.frozen["stamp"]
        m = self.ros.latest_map
        return m["stamp"] if m else None

    def get_map(self, since=None):
        if self.mode == "nav" and self.frozen:
            m = self.frozen
        else:
            raw = self.ros.latest_map
            if raw is None:
                return None
            if self._cache[0] != raw["stamp"]:
                conv = {k: raw[k] for k in ("stamp", "w", "h", "res", "ox", "oy", "yaw")}
                conv["rows"] = occupancy_to_rows(raw["w"], raw["h"], raw["data"])
                self._cache = (raw["stamp"], conv)
            m = self._cache[1]
        if since is not None and since == m["stamp"]:
            return {"unchanged": True, "stamp": m["stamp"]}
        return m

    def get_scan(self):
        sc = getattr(self.ros, "latest_scan", None)
        if not sc or time.time() - sc["ts"] > 1.5:        # 오래된 스캔은 보내지 않는다 (라이다가 멈췄을 때 얼어붙은 화면 방지)
            return {"empty": True}
        return {k: sc[k] for k in ("a0", "da", "lx", "ly", "rmax", "r")}

    def shutdown(self):
        try:
            self.ros.set_drive(0.0, 0.0)
        except Exception:
            pass
        self._stop_nav()
        self._stop_slam()


# ======================================================================
# HTTP
# ======================================================================
def make_handler(core, web_dir):
    index_path = Path(web_dir) / "index.html"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json; charset=utf-8"):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ("/", "/index.html"):
                try:
                    return self._send(200, index_path.read_bytes(), "text/html; charset=utf-8")
                except OSError:
                    return self._send(404, {"error": "index.html 이 없어요"})
            if url.path == "/api/state":
                return self._send(200, core.state())
            if url.path == "/api/map":
                since = parse_qs(url.query).get("since", [None])[0]
                m = core.get_map(since)
                return self._send(200, m if m is not None else {"empty": True})
            if url.path == "/api/scan":
                return self._send(200, core.get_scan())
            self._send(404, {"error": "없는 주소예요"})

        def do_POST(self):
            # JSON 만 받는다: 다른 사이트의 페이지가 몰래 차를 움직이는 요청(단순 폼 전송)을 막는다
            if not self.headers.get("Content-Type", "").lower().startswith("application/json"):
                return self._send(415, {"error": "Content-Type 은 application/json 이어야 해요"})
            try:
                n = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError
            except (ValueError, OSError):
                return self._send(400, {"error": "JSON 형식이 올바르지 않아요"})
            path = urlparse(self.path).path
            try:
                if path == "/api/drive":
                    core.drive(body.get("v", 0), body.get("w", 0))
                    return self._send(200, {"ok": True})
                if path == "/api/plan":
                    return self._send(200, core.preview_goal(body.get("goal")))
                if path == "/api/cmd":
                    core.send_cmd(body.get("cmd"), body.get("points"))
                    return self._send(200, {"ok": True})
                if path == "/api/mapping/start":
                    core.start_mapping()
                    return self._send(200, {"ok": True})
                if path == "/api/mapping/finish":
                    res = core.finish_mapping(body.get("name"), body.get("radius", 0.12), body.get("speed", 0.08))
                    return self._send(200, {"ok": True, **res})
                if path == "/api/nav/restart":
                    core.restart_nav(body.get("radius", 0.12), body.get("speed", 0.08))
                    return self._send(200, {"ok": True})
                if path == "/api/slam/stop":
                    core.stop_all()
                    return self._send(200, {"ok": True})
            except Conflict as exc:
                return self._send(409, {"error": str(exc)})
            except (TypeError, ValueError) as exc:
                return self._send(400, {"error": f"값이 올바르지 않아요: {exc}"})
            self._send(404, {"error": "없는 주소예요"})

    return Handler


def tailscale_ip():
    try:
        out = subprocess.run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=3).stdout.split()
        return out[0] if out else None
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=None, help="열 주소 (기본: Tailscale IPv4, 없으면 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--launch", default="~/ros2_amr_ws/src/riding_test/slam_wheel_odom.launch.py")
    ap.add_argument("--maps-dir", default="~/ros2_amr_ws/maps")
    args = ap.parse_args()

    host = args.host or tailscale_ip() or "127.0.0.1"
    if host in ("0.0.0.0", "::"):
        print("[경고] 모든 네트워크에 열면 같은 망의 누구나 차를 움직일 수 있어요.")
    core = Core(RosBridge(), args.launch, args.maps_dir, HERE / "waypoint_nav_node.py")
    server = ThreadingHTTPServer((host, args.port), make_handler(core, HERE))
    server.daemon_threads = True
    print(f"서버 시작: http://{host}:{args.port}   (종료: Ctrl+C)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        core.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
