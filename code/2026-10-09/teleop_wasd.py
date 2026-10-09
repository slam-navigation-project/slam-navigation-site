#!/usr/bin/env python3
"""
tracker_encoder.py 와 같은 방식의 키보드 조종 (ROS2 /cmd_vel 발행)

W/S: 전진/후진   A/D: 제자리 좌/우회전   Space: 정지   Q: 종료
한 번 누르면 다른 키를 누를 때까지 그 동작을 유지합니다.
명령을 0.1초마다 계속 보내므로 wheel_odom_node의 안전 정지(0.5초)에 걸리지 않습니다.
"""
import curses
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LINEAR_SPEED = 0.05    # 전진 속도 (m/s). 
TURN_SPEED = 0.5       # 제자리 회전 속도 (rad/s)
PUBLISH_PERIOD = 0.1   # 명령 재전송 주기 (s)

COMMANDS = {
    "FORWARD": (LINEAR_SPEED, 0.0),
    "BACKWARD": (-LINEAR_SPEED, 0.0),
    "TURN_LEFT": (0.0, TURN_SPEED),
    "TURN_RIGHT": (0.0, -TURN_SPEED),
    "STOP": (0.0, 0.0),
}
KEYMAP = {
    ord('w'): "FORWARD", ord('W'): "FORWARD",
    ord('s'): "BACKWARD", ord('S'): "BACKWARD",
    ord('a'): "TURN_LEFT", ord('A'): "TURN_LEFT",
    ord('d'): "TURN_RIGHT", ord('D'): "TURN_RIGHT",
    ord(' '): "STOP",
}


class TeleopWASD(Node):
    def __init__(self):
        super().__init__("teleop_wasd")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.state = "STOP"
        self.odom = None

    def on_odom(self, msg):
        self.odom = msg

    def send(self):
        v, w = COMMANDS[self.state]
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)


def main(stdscr):
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    rclpy.init()
    node = TeleopWASD()
    last_send = 0.0
    try:
        while True:
            key = stdscr.getch()
            if key in (ord('q'), ord('Q')):
                break
            if key in KEYMAP:
                node.state = KEYMAP[key]
                last_send = 0.0                     # 키를 바꾸면 바로 전송

            now = time.monotonic()
            if now - last_send >= PUBLISH_PERIOD:
                node.send()
                last_send = now

            rclpy.spin_once(node, timeout_sec=0.0)

            v, w = COMMANDS[node.state]
            stdscr.erase()
            stdscr.addstr(0, 0, "==============================================")
            stdscr.addstr(1, 0, "      키보드 조종 (ROS2 /cmd_vel)             ")
            stdscr.addstr(2, 0, "==============================================")
            stdscr.addstr(4, 0, f" [상태] {node.state:<10} | v {v:+.2f} m/s | ω {w:+.2f} rad/s")
            if node.odom is not None:
                p = node.odom.pose.pose
                yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
                tw = node.odom.twist.twist
                stdscr.addstr(6, 0, f"  x = {p.position.x * 100:>7.1f} cm   y = {p.position.y * 100:>7.1f} cm")
                stdscr.addstr(7, 0, f"  θ = {math.degrees(yaw):>7.1f} deg")
                stdscr.addstr(8, 0, f"  측정 속도 v {tw.linear.x:+.3f} m/s | ω {math.degrees(tw.angular.z):+.1f} deg/s")
            else:
                stdscr.addstr(6, 0, "  /odom 대기 중... (런치가 실행 중인지 확인)")
            stdscr.addstr(10, 0, " [조작] W/S: 전후진 | A/D: 좌우회전 | Space: 정지 | Q: 종료")
            stdscr.refresh()
            time.sleep(0.02)
    finally:
        node.state = "STOP"
        for _ in range(3):                          # 종료할 때 정지 명령을 확실히 전달
            node.send()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    curses.wrapper(main)
