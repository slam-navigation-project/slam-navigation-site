# 2026-10-09 바퀴 오도메트리(엔코더 + 자이로) + SLAM

| 파일 | 역할 |
| --- | --- |
| `tracker_encoder.py` | 하드웨어 라이브러리: 엔코더, MPU6050, 바퀴 속도 제어(피드포워드 + PI), 오도메트리. 단독 실행 시 키보드 주행 |
| `motor_calib.json` | `--calibrate`로 측정한 모터별 PWM-각속도 표 |
| `wheel_odom_node.py` | ROS2 노드: `/cmd_vel` 구독, `/odom` 및 TF `odom → base_footprint` 발행 |
| `teleop_wasd.py` | 키보드 조종 (W/A/S/D, Space 정지, Q 종료) |
| `slam_wheel_odom.launch.py` | 라이다, static TF, 오도메트리 노드, slam_toolbox, rosbridge 실행 |
| `slam_custom_params.yaml` | slam_toolbox 파라미터 |

## 실행 (Raspberry Pi, 모든 파일을 홈 폴더 `~/`에 둠)

```bash
python3 ~/tracker_encoder.py --calibrate      # 최초 1회, 바퀴 띄우고
python3 ~/tracker_encoder.py                  # 단독 주행 테스트

ros2 launch ~/slam_wheel_odom.launch.py       # 터미널 1
python3 ~/teleop_wasd.py                      # 터미널 2
ros2 run tf2_ros tf2_echo map odom            # 보정량 확인

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rpi/my_map'}}"
```

설명: 블로그 2026-10-09 글 3편 (엔코더 / 자이로 / 오도메트리 + SLAM)
