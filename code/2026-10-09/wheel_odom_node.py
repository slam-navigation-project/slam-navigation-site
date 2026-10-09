#!/usr/bin/env python3
"""
엔코더 + 자이로 바퀴 오도메트리 ROS2 노드 (laser_odom.py 대체)

- 구독: /cmd_vel (geometry_msgs/Twist)  -> 바퀴 속도 제어
- 발행: /odom (nav_msgs/Odometry), TF odom -> base_footprint

하드웨어 제어(엔코더, MPU6050, 바퀴 속도 PI, 오도메트리 계산)는
같은 폴더의 tracker_encoder.py에 있는 Robot 클래스를 그대로 사용합니다.
시작할 때 자이로 영점을 잡으므로 약 1초 동안 차를 움직이지 마세요.
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tracker_encoder as hw   # noqa: E402  (같은 폴더의 tracker_encoder.py)

CMD_TIMEOUT = 0.5      # 이 시간(s) 동안 /cmd_vel이 안 오면 정지 (통신 끊김 대비)
PUBLISH_RATE = 30.0    # /odom, TF 발행 주기 (Hz)
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__("wheel_odom")
        self.get_logger().info("자이로 영점 잡는 중... 차를 움직이지 마세요")
        self.robot = hw.Robot()
        self.get_logger().info(
            f"시작: 자이로 {'ON' if self.robot.imu else 'OFF'}, "
            f"모터보정 {'ON' if self.robot.calib else 'OFF'}")

        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / PUBLISH_RATE, self.on_timer)

    def on_cmd_vel(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.robot.set_velocity(msg.linear.x, msg.angular.z)

    def on_timer(self):
        now = self.get_clock().now()

        # 명령이 끊기면 정지
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > CMD_TIMEOUT:
            self.robot.set_velocity(0.0, 0.0)

        r = self.robot
        x, y, th = r.x, r.y, r.theta
        qx, qy, qz, qw = yaw_to_quaternion(th)

        # 현재 속도: 전진은 엔코더, 회전은 자이로(있으면)
        v = hw.WHEEL_RADIUS * (r.omega_l + r.omega_r) / 2.0
        if r.imu is not None:
            w = math.radians(r.gyro_dps)
        else:
            w = hw.WHEEL_RADIUS * (r.omega_r - r.omega_l) / hw.TRACK_WIDTH

        stamp = now.to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = ODOM_FRAME
        o.child_frame_id = BASE_FRAME
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        # 대략적인 불확실성 (x, y, yaw만 의미 있음. 나머지는 큰 값)
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.01      # x, y (m²)
        pc[35] = 0.02             # yaw (rad²)
        pc[14] = pc[21] = pc[28] = 1e6
        o.pose.covariance = pc
        tc = [0.0] * 36
        tc[0] = 0.005
        tc[35] = 0.01
        tc[7] = tc[14] = tc[21] = tc[28] = 1e6
        o.twist.covariance = tc
        self.odom_pub.publish(o)

    def destroy_node(self):
        self.robot.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass   # Ctrl+C, kill, 런치 종료 모두 여기로
    finally:
        try:
            node.destroy_node()        # 모터 정지, 엔코더/자이로 정리
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            os._exit(0)                # 남은 스레드가 있어도 확실히 종료해서 GPIO를 놓음


if __name__ == "__main__":
    main()
