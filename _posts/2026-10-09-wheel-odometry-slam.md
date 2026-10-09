---
layout: post
title: "바퀴 오도메트리(엔코더 + 자이로) 기반 SLAM 매핑 (진행 중)"
date: 2026-10-09 21:00:00 +0900
categories: [Capstone, ROS2, AMR]
tags: [ROS2-Humble, SLAM-Toolbox, Wheel-Odometry, TF, RPLIDAR, Foxglove]
---

## 1. 진행 개요

- 기존 `laser_odom.py`(라이다 점군만으로 이동량 추정)는 오차가 너무 커서, **엔코더 + 자이로 바퀴 오도메트리**로 교체.
- ROS2 노드(`wheel_odom_node.py`)로 `/odom`과 `odom → base_footprint` TF 발행.
- 키보드 조종 노드(`teleop_wasd.py`)로 `/cmd_vel` 전송.
- `slam_toolbox`로 방 지도 생성. 오차를 하나씩 잡아 가며 지도가 점점 깨끗해지는 중이며, **아직 진행 중**이다.

> 이전 글: [엔코더 기반 바퀴 속도 제어]({% post_url 2026-10-09-encoder-speed-control %}),
> [MPU6050 자이로로 방향 유지하기]({% post_url 2026-10-09-mpu6050-gyro-heading %})

---

## 2. 전체 구조

```text
[PC]                                  [Raspberry Pi]
teleop_wasd.py ──/cmd_vel──▶ wheel_odom_node.py ──▶ 모터 (PI 속도 제어)
                                     │
                                     ├── /odom, TF odom → base_footprint (엔코더 + 자이로)
                                     │
RPLIDAR C1 ──────/scan─────▶ slam_toolbox ──▶ /map, TF map → odom
                                     │
Foxglove ◀────rosbridge(9090)────────┘
```

### 파일 구성

| 파일 | 역할 |
| --- | --- |
| `tracker_encoder.py` | 하드웨어 라이브러리 (엔코더, 자이로, 속도 제어, 오도메트리) |
| `wheel_odom_node.py` | ROS2 노드: `/cmd_vel` 구독, `/odom` 및 TF 발행 |
| `slam_wheel_odom.launch.py` | 라이다, static TF, 오도메트리 노드, slam_toolbox, rosbridge 실행 |
| `slam_custom_params.yaml` | slam_toolbox 파라미터 |
| `teleop_wasd.py` | 키보드 조종 (W/A/S/D, Space 정지) |

---

## 3. TF 구조

```text
map ──(slam_toolbox)──▶ odom ──(wheel_odom_node)──▶ base_footprint ──(static)──▶ laser
```

| TF | 발행 주체 | 의미 |
| --- | --- | --- |
| `map → odom` | slam_toolbox | 오도메트리 누적 오차 보정량 |
| `odom → base_footprint` | wheel_odom_node | 엔코더 + 자이로로 계산한 차 위치 |
| `base_footprint → laser` | static_transform_publisher | 라이다 장착 위치 |

`map → odom`은 **오도메트리가 얼마나 틀렸는지**를 보여 준다. 오도메트리가 정확할수록 이 값이 0 근처에 머문다.

```bash
ros2 run tf2_ros tf2_echo map odom
```

---

## 4. 코드 설명 (중요 부분)

### (1) `wheel_odom_node.py`: 하드웨어와 ROS2 연결

하드웨어 제어는 1, 2편의 `tracker_encoder.py`를 그대로 `import`해서 쓰고, 이 노드는 **ROS2 메시지 ↔ `Robot` 클래스** 연결만 담당한다.

```python
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tracker_encoder as hw
...
self.robot = hw.Robot()
```

**왜 이렇게 했나**: 단독 키보드 주행(`tracker_encoder.py`)과 ROS2 주행이 **같은 제어 코드**를 쓰게 하기 위해서다. 속도 제어나 자이로 파라미터를 한 곳에서만 고치면 둘 다 반영된다.

#### `/cmd_vel` → 바퀴 목표 속도

```python
def on_cmd_vel(self, msg):
    self.last_cmd_time = self.get_clock().now()
    self.robot.set_velocity(msg.linear.x, msg.angular.z)
```

`set_velocity()`는 차체 속도 `(v, ω)`를 차동 구동 식으로 바퀴 각속도로 바꾼다.

```python
wl = (v - omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
wr = (v + omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
peak = max(abs(wl), abs(wr))
if peak > MAX_WHEEL_OMEGA:               # 한쪽이 한계를 넘으면 비율을 유지한 채 둘 다 줄임
    wl *= MAX_WHEEL_OMEGA / peak
    wr *= MAX_WHEEL_OMEGA / peak
```

- 한계를 넘을 때 각 바퀴를 따로 자르면 좌우 비율이 바뀌어 **곡률(도는 정도)이 달라진다.** 비율을 유지한 채 함께 줄여서 경로 모양은 유지하고 속도만 느려지게 하였다.
- `/cmd_vel`은 ROS2 표준 명령이라, 나중에 A* 경로 추종 노드도 같은 토픽으로 명령을 보내면 된다.

#### 안전 정지

```python
if (now - self.last_cmd_time).nanoseconds * 1e-9 > CMD_TIMEOUT:   # 0.5초
    self.robot.set_velocity(0.0, 0.0)
```

조종 프로그램이 꺼지거나 Wi-Fi가 끊기면 마지막 명령이 계속 유지되어 차가 벽으로 돌진할 수 있다.  
0.5초 동안 명령이 없으면 스스로 멈춘다.

#### `/odom`, TF 발행 (30Hz)

```python
v = hw.WHEEL_RADIUS * (r.omega_l + r.omega_r) / 2.0
if r.imu is not None:
    w = math.radians(r.gyro_dps)       # 회전 속도는 자이로
...
self.tf_broadcaster.sendTransform(t)  # odom → base_footprint
self.odom_pub.publish(o)              # /odom
```

- 위치(x, y, θ)는 `Robot`이 이미 계산해 둔 값을 그대로 보낸다.
- slam_toolbox가 실제로 쓰는 것은 **TF `odom → base_footprint`** 이다. `/odom` 토픽은 화면 표시와 이후 내비게이션용이다.
- 공분산(covariance)에는 x, y, yaw만 의미 있는 값을 넣고, 2D 로봇이 움직일 수 없는 z, roll, pitch에는 아주 큰 값을 넣었다.

#### 확실한 종료

```python
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()        # 모터 정지, 엔코더/자이로 정리
    os._exit(0)                # 남은 스레드가 있어도 확실히 종료해서 GPIO를 놓음
```

런치를 Ctrl+C로 끄면 노드는 SIGTERM을 받는다. 이 경우를 처리하지 않으면 프로세스가 남아 **GPIO를 계속 점유**하고, 다음 실행 때 `GPIO busy`가 났다 (5장).

### (2) `teleop_wasd.py`: 키보드 조종

```python
COMMANDS = {
    "FORWARD": (LINEAR_SPEED, 0.0),      # 0.05 m/s
    "TURN_LEFT": (0.0, TURN_SPEED),      # 0.5 rad/s
    ...
}
...
if key in KEYMAP:
    node.state = KEYMAP[key]
    last_send = 0.0                      # 키를 바꾸면 바로 전송
if now - last_send >= PUBLISH_PERIOD:    # 0.1초마다 재전송
    node.send()
```

**왜 새로 만들었나**

- 기본 `teleop_twist_keyboard`는 키를 누를 때마다 속도가 단계적으로 올라가고, 키 반복 입력 때문에 꾹 누르면 점점 빨라졌다.
- `tracker_encoder.py` 단독 실행 때처럼 **키 한 번 = 동작 고정, Space = 정지** 방식으로 맞췄다.
- **0.1초마다 재전송**하는 이유: 노드의 안전 정지(0.5초)에 걸리지 않게 하기 위해서다. 같은 명령이 반복해서 들어와도 `set_velocity()`가 무시하므로 방향 유지 기준이 흐트러지지 않는다 (2편).
- 종료 시 정지 명령을 3번 보내서, 마지막 명령이 남아 차가 계속 가는 일을 막았다.

### (3) `slam_wheel_odom.launch.py`: 한 번에 실행

| 순서 | 실행 대상 | 비고 |
| --- | --- | --- |
| 1 | `rplidar_node` | `/dev/ttyUSB0`, 460800 baud, `frame_id: laser` |
| 2 | static TF `base_footprint → laser` | 라이다 장착 위치 (`LASER_X/Y/Z/YAW`) |
| 3 | `wheel_odom_node.py` | 패키지가 아닌 단일 스크립트라 `ExecuteProcess`로 실행 |
| 4 | `slam_toolbox` (async) | `~/slam_custom_params.yaml` 사용 |
| 5 | `rosbridge_websocket` | Foxglove 연결용 (9090 포트) |

```python
LASER_X = '0.0'        # 바퀴 축 중심 기준 앞(+)/뒤(-)
LASER_Y = '0.0'
LASER_Z = '0.10'       # 바닥에서 스캔 면까지 높이
LASER_YAW = '3.14159'  # 드라이버의 laser +x가 라이다 화살표 반대 방향 (5장)
```

`base_footprint`는 **두 바퀴 축의 가운데, 바닥 높이**로 정하였다. 차동 구동 차량은 이 점을 중심으로 회전하기 때문이다.  
라이다가 이 점에서 떨어져 있으면, 회전할 때 라이다는 원을 그리며 움직이므로 그 거리를 정확히 넣어야 한다.

### (4) `slam_custom_params.yaml`: 주요 파라미터

| 파라미터 | 값 | 왜 |
| --- | --- | --- |
| `odom_frame` / `base_frame` | `odom` / `base_footprint` | 바퀴 오도메트리 노드의 TF 이름과 일치 |
| `resolution` | 0.05 | 지도 한 칸 5cm. 차 크기(약 15cm) 대비 적당하고 라즈베리파이 부담이 작음 |
| `min_laser_range` | 0.2 | 20cm 안쪽은 **차체 자기 자신**(배선, 기둥)이 찍히므로 무시 |
| `max_laser_range` | 5.0 | 먼 거리 점은 각도 오차가 크게 벌어져 노이즈가 됨. 방 안에서는 5m면 충분 |
| `minimum_travel_distance` / `heading` | 0.10 m / 0.10 rad | 10cm 또는 약 6° 움직였을 때만 새 스캔을 지도에 추가. 너무 작으면 노드가 많아져 느려지고 오차가 쌓임 |
| `map_update_interval` | 1.0 | 지도 이미지 갱신 주기. 라즈베리파이 CPU 부담 감소 |
| `use_scan_matching` | true | 오도메트리 위치를 스캔 매칭으로 한 번 더 보정 (이 보정량이 `map → odom`) |
| `do_loop_closing` | true | 한 바퀴 돌아 처음 장소에 오면 누적 오차를 한 번에 정리 |

> 예전 `laser_odom.py`를 쓸 때는 손으로 들고 다녀서 이동 임계값을 아주 작게 잡았었다. 바퀴 오도메트리로 바꾸면서 일반적인 값(10cm)으로 되돌렸다.

### (5) 실행

```bash
# 터미널 1 (라즈베리파이)
ros2 launch ~/slam_wheel_odom.launch.py

# 터미널 2 (라즈베리파이)
python3 ~/teleop_wasd.py

# 보정량 확인
ros2 run tf2_ros tf2_echo map odom

# 지도 저장
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rpi/my_map'}}"
```

---

## 5. 트러블슈팅

### (1) 라이다 정면 방향 (LASER_YAW)

가장 오래 걸린 문제였다. 지도가 계속 번지고 벽이 이중으로 겹쳤다.

- 라이다 몸체에 그려진 화살표 방향과, `rplidar_ros` 드라이버가 발행하는 `laser` 프레임의 +x 방향이 **반대**였다.
- 그래서 차가 앞으로 가면 스캔은 뒤로 가는 것처럼 보였고, slam_toolbox가 오도메트리와 스캔을 맞추지 못하였다.

**확인 방법**: Foxglove에서 Fixed frame을 `odom`으로 두고, 라이다 앞에 휴대폰을 대 보았을 때 점이 `base_footprint`의 빨간 화살표(+x) 쪽에 찍히는지 확인.

**해결**: 런치 파일의 static TF에 180° 회전을 넣었다.

```python
LASER_YAW = '3.14159'   # 라이다 화살표가 차 앞을 보고 있어도 드라이버 기준으로는 뒤
```

> 처음에 이 값을 `slam_custom_params.yaml`에 넣었는데, yaml은 slam_toolbox 파라미터 파일이라 적용되지 않았다. **런치 파일**에 넣어야 한다.

### (2) GPIO busy / Address already in use

이전 런치가 완전히 종료되지 않고 노드가 남아 GPIO와 rosbridge 포트(9090)를 계속 잡고 있었다.

```bash
pkill -f wheel_odom_node; pkill -f rosbridge
```

노드가 종료 신호(SIGTERM)를 받으면 확실히 끝나도록 수정하였다.

```python
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()      # 모터 정지, GPIO 해제
    os._exit(0)
```

### (3) 디스크 부족 (`OSError: [Errno 28]`)

journal 로그, snap 이전 버전, 사용하지 않는 프로그램, 캐시를 정리하고  
ROS 로그를 `/tmp`로 옮겨 재부팅 시 자동 삭제되도록 하였다.

```bash
sudo journalctl --vacuum-size=100M
echo 'export ROS_LOG_DIR=/tmp/ros_log' >> ~/.bashrc
```

---

## 6. 지도 생성 과정

오차를 하나씩 잡아 가며 같은 방을 여러 번 매핑하였다.

### (1) 초기: 벽이 이중으로 겹침

오도메트리와 스캔이 맞지 않아 같은 벽이 회전된 채로 두 번 그려졌다.

![SLAM 지도 1 - 벽 겹침]({{ site.baseurl }}/assets/img/posts/2026-10-09/2026-10-09-slam-map-1.png)

### (2) 스캔이 방사형으로 번짐

이동, 회전할 때마다 스캔이 다른 위치에 찍히면서 지도가 크게 번졌다.

![SLAM 지도 2 - 스캔 번짐]({{ site.baseurl }}/assets/img/posts/2026-10-09/2026-10-09-slam-map-2.png)

### (3) 윤곽이 하나로 모이기 시작

라이다 방향(LASER_YAW) 등 오차를 하나씩 잡으면서 방 윤곽이 한 겹으로 모이기 시작하였다.

![SLAM 지도 3 - 윤곽 정리]({{ site.baseurl }}/assets/img/posts/2026-10-09/2026-10-09-slam-map-3.png)

### (4) 현재

방 한 바퀴를 돌았을 때의 지도. 벽 겹침은 거의 사라지고 방 형태를 알아볼 수 있는 수준이 되었다.

![SLAM 지도 4 - 현재]({{ site.baseurl }}/assets/img/posts/2026-10-09/2026-10-09-slam-map-4.png)

`tf2_echo map odom`으로 확인한 보정량은 다음과 같다.

| 항목 | 보정량 | 판단 |
| --- | --- | --- |
| 회전 (θ) | 약 −5° ~ +4° | 양호 (자이로가 정확함) |
| 거리 (x, y) | 최대 약 0.2 ~ 0.5 m | 큼 → 거리 오차가 남아 있음 |

---

## 7. 현황 및 남은 문제 (진행 중)

### (1) 남은 문제

- 벽이 두껍고 들쭉날쭉함, 벽 바깥으로 빈 공간(흰색)이 새어 나가는 부분 존재.
- 회전 보정은 작지만 **거리 보정이 이동할수록 한 방향으로 쌓임** → 바퀴 반지름(거리 배율) 오차 또는 바퀴 미끄러짐.
  (위치는 각속도가 아니라 엔코더 카운트로 계산하므로 바퀴 속도가 일정하지 않은 것 자체는 원인이 아니다. [1편 8장]({% post_url 2026-10-09-encoder-speed-control %}) 참고)
- 현재 차체가 단순 박스 형태라 **앞으로 기울어짐** → 라이다가 수평이 아니고, 바퀴 접지도 불안정.
- 방 안의 작은 점들은 대부분 의자 다리, 가방 같은 실제 물체 (라이다 높이 약 10cm).

### (2) 다음 할 일

1. **차체 재제작**: 라이다를 바퀴 축 중심 위에 수평으로 장착, 무게중심을 바퀴 축 근처로, 보조 바퀴 추가, 자이로 단단히 고정.
2. 새 차체 기준으로 재보정
   - `python3 ~/tracker_encoder.py --calibrate` (바퀴 띄운 상태)
   - 1m 주행 테스트로 `WHEEL_RADIUS` 보정
   - `TRACK_WIDTH`, `LASER_Z` 재측정
3. 회전 속도(`TURN_SPEED`) 낮추기, 회전 후 잠깐 멈췄다 출발하기, 출발점으로 돌아와 루프 닫기.
   (`max_laser_range: 5.0`, `ACCEL_LIMIT = 3.0`은 적용 완료)
4. 깨끗한 지도 저장 후, 기존에 테스트한 **A\* 경로 탐색**과 연결.
