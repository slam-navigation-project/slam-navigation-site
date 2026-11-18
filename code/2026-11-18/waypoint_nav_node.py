#!/usr/bin/env python3
"""
웨이포인트 왕복 주행 노드 (A* + 경로 추종)

입력 (ROS 토픽)
  /clicked_point    geometry_msgs/PointStamped  웨이포인트 1개 추가 (map 프레임). Foxglove 3D 패널의 Publish point 도구가 이 토픽으로 보낸다.
  /nav_enable       std_msgs/Bool               true: 주행 시작, false: 정지
  /waypoints_clear  std_msgs/Empty              웨이포인트 전부 지우고 정지

출력
  /cmd_vel          geometry_msgs/Twist         주행 중일 때만 10Hz로 발행 (멈춰 있을 땐 발행 안 함 -> 텔레옵과 안 부딪힘)
  /astar_path       nav_msgs/Path               지금 따라가는 경로
  /waypoints        visualization_msgs/MarkerArray  찍은 웨이포인트

필요: slam_toolbox가 map->odom TF를 내고 있어야 함 (map 프레임 기준 차 위치를 TF로 읽는다).
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from astar_planner import GridMap, plan          # noqa: E402
from path_follower import PathFollower, PingPong  # noqa: E402

MAP_FRAME = "map"
BASE_FRAME = "base_footprint"
CONTROL_PERIOD = 0.1   # s
CMD_FILE = "/tmp/amr_cmd.json"      # PC GUI가 SFTP로 써 넣는 명령 (go / stop / clear)
STATE_FILE = "/tmp/amr_state.json"  # 노드가 쓰는 상태 (PC GUI가 읽어서 로봇/경로를 그림)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointNav(Node):
    def __init__(self):
        super().__init__("waypoint_nav")
        self.declare_parameter("map_yaml", os.path.expanduser("~/maps/room_map.yaml"))
        self.declare_parameter("map_npz", "")
        self.declare_parameter("robot_radius", 0.12)     # 차 반경(m) + 여유. 실제 차 크기에 맞게 조정
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("rotate_threshold", 0.5)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("vertex_tolerance", 0.10)

        p = self.get_parameter
        map_source = p("map_npz").value or p("map_yaml").value
        self.grid = GridMap(map_source, robot_radius=p("robot_radius").value)
        self.get_logger().info(
            f"맵 로드: {self.grid.width}x{self.grid.height}칸, 반경 {p('robot_radius').value} m 팽창")

        self.follower = PathFollower(
            linear_speed=p("linear_speed").value, turn_speed=p("turn_speed").value,
            rotate_threshold=p("rotate_threshold").value,
            vertex_tol=p("vertex_tolerance").value, goal_tol=p("goal_tolerance").value)
        self.route = PingPong()
        self.active = False
        self.has_path = False
        self.current_path = []
        self.fail_count = 0
        self.msg = ""
        self.tick = 0
        self.last_seq = None      # 마지막으로 처리한 명령 번호
        self.cmd_mtime = 0.0
        # 노드가 켜지기 전에 남은 옛 명령은 무시한다 (현재 파일의 seq를 이미 처리한 것으로 간주)
        old = self.read_cmd_file()
        if old is not None:
            self.last_seq = old.get("seq")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/astar_path", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoints", 1)
        self.create_subscription(PointStamped, "/clicked_point", self.on_point, 10)
        self.create_subscription(Bool, "/nav_enable", self.on_enable, 10)
        self.create_subscription(Empty, "/waypoints_clear", self.on_clear, 10)
        self.create_timer(CONTROL_PERIOD, self.on_timer)

    # ---------- 입력 ----------
    def add_waypoint(self, x, y):
        """갈 수 있는 곳 근처면 웨이포인트로 추가하고 True. 아니면 False."""
        cell = self.grid.world_to_cell(x, y)
        if not self.grid.is_free(*cell):
            self.say(f"({x:.2f}, {y:.2f})의 셀이 통행 불가여서 목적지를 거부", warn=True)
            return False
        self.route.add((x, y))
        self.say(f"웨이포인트 #{len(self.route.points) - 1} 추가: ({x:.2f}, {y:.2f})")
        return True

    def say(self, text, warn=False):
        """로그 + GUI에 보여줄 메시지(상태 파일)를 함께 남긴다."""
        self.msg = text
        (self.get_logger().warn if warn else self.get_logger().info)(text)

    def on_point(self, msg):
        if msg.header.frame_id and msg.header.frame_id != MAP_FRAME:
            self.get_logger().warn(f"'{msg.header.frame_id}' 프레임 점은 무시 (map 프레임만 사용)")
            return
        self.stop("목적지 변경")
        self.route.clear()
        self.add_waypoint(msg.point.x, msg.point.y)
        self.publish_markers()

    def start_route(self):
        if len(self.route.points) < 1:
            self.say("웨이포인트가 없어 시작 안 함", warn=True)
            return
        self.route.i = 0
        self.route.dir = 1
        self.has_path = False
        self.active = True
        self.say("주행 시작")

    def on_enable(self, msg):
        if msg.data:
            self.start_route()
        else:
            self.stop("정지 명령")

    # ---------- PC GUI 연동 (파일) ----------
    def read_cmd_file(self):
        try:
            with open(CMD_FILE) as f:
                return json.load(f)
        except Exception:
            return None

    def poll_cmd_file(self):
        try:
            mtime = os.path.getmtime(CMD_FILE)
        except OSError:
            return
        if mtime == self.cmd_mtime:
            return
        self.cmd_mtime = mtime
        cmd = self.read_cmd_file()
        if not isinstance(cmd, dict) or cmd.get("seq") == self.last_seq:
            return
        self.last_seq = cmd.get("seq")
        kind = cmd.get("cmd")
        if kind == "go":        # 웨이포인트 전체 교체 후 시작
            self.stop("새 웨이포인트로 교체")
            self.route.clear()
            # 관제에서는 도착지 1개만 받음 (왕복 웨이포인트는 사용하지 않음).
            for p in cmd.get("points", [])[:1]:
                try:
                    self.add_waypoint(float(p[0]), float(p[1]))
                except (TypeError, ValueError, IndexError):
                    self.say(f"잘못된 웨이포인트 형식: {p}", warn=True)
            self.publish_markers()
            self.start_route()
        elif kind == "stop":
            self.stop("정지 명령 (GUI)")
        elif kind == "clear":
            self.stop("웨이포인트 초기화 (GUI)")
            self.route.clear()
            self.publish_markers()
        else:
            self.say(f"알 수 없는 명령: {kind}", warn=True)

    def write_state(self, pose):
        state = {
            "time": time.time(),
            "active": self.active,
            "pose": list(pose) if pose else None,
            "target_index": self.route.i if self.route.points else None,
            "waypoints": [list(p) for p in self.route.points],
            "path": [list(p) for p in self.current_path] if self.active else [],
            "msg": self.msg,
        }
        tmp = STATE_FILE + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, STATE_FILE)      # 읽는 쪽이 반쯤 쓴 파일을 보지 않도록 한 번에 교체
        except OSError:
            pass

    def on_clear(self, _):
        self.stop("웨이포인트 초기화")
        self.route.clear()
        self.publish_markers()

    def stop(self, why):
        if self.active:
            self.say(why)
        self.active = False
        self.has_path = False
        self.fail_count = 0
        for _ in range(3):
            self.cmd_pub.publish(Twist())

    # ---------- 위치 ----------
    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time(),
                                                timeout=Duration(seconds=0.05))
        except Exception:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation)

    # ---------- 주행 ----------
    def plan_to_current_target(self, x, y):
        target = self.route.current()
        path = plan(self.grid, (x, y), target)
        if path is None:
            self.say(f"({target[0]:.2f}, {target[1]:.2f})로 가는 경로가 없어 다음 목표로 넘어감", warn=True)
            return False
        self.fail_count = 0
        self.current_path = path
        self.follower.set_path(path)
        self.publish_path(path)
        self.has_path = True
        return True

    def on_timer(self):
        self.poll_cmd_file()
        pose = self.get_pose()
        self.tick += 1
        if self.tick % 2 == 0:                 # 5Hz로 상태 파일 갱신 (멈춰 있어도 GUI가 차 위치를 볼 수 있게)
            self.write_state(pose)
        if not self.active:
            return
        if pose is None:
            self.get_logger().warn("map->base_footprint TF 없음 (SLAM 실행 중인지 확인)",
                                   throttle_duration_sec=2.0)
            return
        x, y, yaw = pose

        if not self.has_path:
            if not self.plan_to_current_target(x, y):
                self.fail_count += 1
                if len(self.route.points) <= 1 or self.fail_count >= len(self.route.points):
                    self.stop("갈 수 있는 목표가 없어 정지")
                    return
                self.route.advance()
                return

        v, w, done = self.follower.step(x, y, yaw)
        if done:
            self.say(f"웨이포인트 #{self.route.i} 도착")
            self.route.advance()
            self.has_path = False
            if len(self.route.points) <= 1:
                self.stop("단일 웨이포인트 도착, 종료")
            return
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # ---------- 시각화 ----------
    def publish_path(self, pts):
        msg = Path()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for px, py in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = px, py
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (px, py) in enumerate(self.route.points):
            m = Marker()
            m.header.frame_id = MAP_FRAME
            m.ns, m.id = "wp", i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 1.0
            arr.markers.append(m)
            t = Marker()
            t.header.frame_id = MAP_FRAME
            t.ns, t.id = "wp_text", i
            t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = px, py, 0.2
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = str(i)
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = WaypointNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop("종료")        # SIGINT로 컨텍스트가 먼저 닫혔으면 발행이 실패할 수 있음
        except Exception:            # (그래도 wheel_odom_node가 0.5초 안에 /cmd_vel이 끊기면 스스로 정지)
            pass
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
