---
layout: post
title: "ROS2 노드 구조 정리: wheel_odom_node의 스레드와 /odom, TF"
date: 2026-10-12 18:00:00 +0900
categories: [Capstone, ROS2, AMR]
tags: [ROS2-Humble, SLAM-Toolbox, Wheel-Odometry, TF, Threading, rclpy]
---

## 1. 진행 개요

- `wheel_odom_node.py`가 정확히 뭘 하는 노드인지, `/cmd_vel`을 받아서 `/odom`과 TF를 내보내기까지 내부에서 어떤 일이 일어나는지 정리.
- 특히 **스레드가 몇 개 돌고 있는지**, **메인 스레드와 Robot 내부 스레드가 어떻게 역할을 나누는지**, **토픽과 TF가 왜 따로 있는지**를 중심으로 구조를 다시 짚어봤다.
- 피드포워드 보정에 쓰는 `motor_calib.json` 측정값도 표로 정리했다.

> 이전 글: [엔코더 기반 바퀴 속도 제어]({% post_url 2026-10-09-encoder-speed-control %}),
> [MPU6050 자이로로 방향 유지하기]({% post_url 2026-10-09-mpu6050-gyro-heading %}),
> [바퀴 오도메트리 기반 SLAM 매핑]({% post_url 2026-10-09-wheel-odometry-slam %})

---

## 2. 전체 흐름

```text
[PC]                                   [Raspberry Pi]
teleop_wasd.py ──/cmd_vel──▶ wheel_odom_node.py ──▶ 모터 (PI 속도 제어)
                                       │
                                       ├── /odom, TF odom → base_footprint
                                       │        (엔코더 + 자이로)
RPLIDAR C1 ──────/scan─────▶ slam_toolbox ──▶ /map, TF map → odom
                                       │
                                  Foxglove ◀── rosbridge(9090)
```

- `teleop_wasd.py`(PC)는 계산을 하지 않는다. 키 입력에 맞는 `(v, ω)` 값을 미리 정해둔 딕셔너리에서 꺼내 `/cmd_vel`로 보낼 뿐이다.
- `wheel_odom_node.py`(RPi)는 ROS2 노드이고, 1편~3편에서 만든 `tracker_encoder.py`의 `Robot` 클래스를 **import**해서 하드웨어를 제어한다. 실제 계산(엔코더, 자이로, PI, 오도메트리)은 전부 `tracker_encoder.py` 쪽에 있고, 이 노드는 ROS 메시지와 `Robot` 객체를 이어주는 역할만 한다.
- `slam_toolbox`는 `/scan`과 TF를 구독해서 지도를 만들고 위치를 보정한다.

---

## 3. 노드로 실행될 때 vs 단독 실행될 때

`tracker_encoder.py`는 ROS와 무관한 순수 파이썬 모듈이라서 실행 방법이 두 가지다.

| 실행 방법 | 실행 주체 | ROS 연동 |
|---|---|---|
| `ros2 launch ~/slam_wheel_odom.launch.py` | launch가 `python3 wheel_odom_node.py`를 띄움 → 그 안에서 `tracker_encoder`를 import해서 `Robot()` 생성 | O |
| `python3 ~/tracker_encoder.py` | 터미널에서 직접 실행 (`main()` → 관제 데모 화면 + 키보드 주행) | X |
| `python3 ~/tracker_encoder.py --calibrate` | 터미널에서 직접 실행 (`calibrate()`) | X |

같은 `Robot` 클래스를 두 경로에서 그대로 재사용하기 때문에, PI 게인이나 자이로 설정을 한 곳만 고치면 단독 실행과 ROS 실행 양쪽에 다 반영된다.

---

## 4. 프로세스 안에서 스레드 3개가 동시에 돈다

`python3 wheel_odom_node.py` 하나의 프로세스 안에 스레드가 3개 있다.

```text
프로세스 시작
 │
 ├─ ① 메인 스레드 (OS가 자동 생성)
 │     WheelOdomNode() 생성
 │       └─ hw.Robot() 생성 ──┬─ ② 제어 스레드 시작 (daemon)
 │                            └─ ③ 자이로 스레드 시작 (daemon)
 │     rclpy.spin(node)  ← 메인 스레드는 여기서 이벤트 대기 모드로 전환
 │
 ├─ ② 제어 스레드 (Robot._loop)   — 50 Hz, 계속 반복
 │     엔코더 읽기 → 오도메트리(x,y,θ) 갱신 → PWM 계산(피드포워드+PI) → 모터 구동
 │
 └─ ③ 자이로 스레드 (Robot._gyro_loop) — 200 Hz, 계속 반복
       MPU6050 z축 각속도 읽어서 각도 적분
```

| 스레드 | 역할 | 언제 일하나 |
|---|---|---|
| ① 메인 (ROS) | `on_cmd_vel`, `on_timer` 콜백 실행 | 이벤트가 올 때만 |
| ② 제어 | PWM 계산 · 모터 구동 · 오도메트리 갱신 | 쉬지 않고 계속 (50 Hz) |
| ③ 자이로 | 각도 적분 | 쉬지 않고 계속 (200 Hz) |

②③은 `Robot()`이 생성되는 시점(노드 초기화)부터 노드가 종료될 때(`robot.close()`)까지 ROS 이벤트와 무관하게 자기 할 일만 반복한다. 메인 스레드는 그 결과값(`robot.x` 등)을 필요할 때 읽어가기만 한다 — 서로 "기다렸다 맞춰서" 주고받는 게 아니라, 한쪽은 계속 갱신하고 한쪽은 가끔 와서 최신값을 가져가는 식이다.

---

## 5. `/cmd_vel`을 받았을 때 실제로 벌어지는 일

콜백 하나가 모든 걸 처리하는 게 아니라, 세 단계가 서로 다른 주기로 독립 실행된다.

```text
① 키보드 입력
      ↓
② teleop_wasd.py: /cmd_vel 발행
      ↓
③ wheel_odom_node.on_cmd_vel() 콜백 (메인 스레드)
      robot.set_velocity(v, ω)
      → self.target 값만 세팅, PWM 계산은 하지 않음

────────────────────────────────────────
④ Robot._loop() — 제어 스레드가 50 Hz로 "항상" 반복 중
      self.target을 보고 가속 제한(ACCEL_LIMIT)으로 서서히 cmd 변경
      → 보정 방식 선택(직진/곡선/회전) → PI 제어 → PWM 출력
      → 엔코더·자이로로 x,y,θ 갱신

────────────────────────────────────────
⑤ wheel_odom_node.on_timer() — 메인 스레드가 30 Hz 타이머로 "항상" 반복 중
      robot.x, y, θ 를 읽어와서
      → /odom 토픽 발행 + TF(odom→base_footprint) 발행
      ↓
⑥ Foxglove: /odom, TF 구독 → 화면 표시
   slam_toolbox: TF 구독 → 스캔 매칭 → /map, TF(map→odom) 갱신
```

`on_cmd_vel`이 직접 PWM을 계산하거나 토픽을 발행하지 않는다는 게 핵심이다. "목표값 세팅"과 "그 목표를 보고 계속 제어 + 발행하는 일"이 분리되어 있다.

---

## 6. `/odom` 토픽과 TF, 왜 둘 다 내보내나

```python
self.tf_broadcaster.sendTransform(t)   # TF
self.odom_pub.publish(o)               # /odom 토픽
```

| | `/odom` 토픽 | TF (`odom→base_footprint`) |
|---|---|---|
| 정체 | 이름 붙은 일반 방송 채널 | 좌표계 간 위치 관계 전담 시스템 |
| 담기는 값 | 위치 + 방향 + **속도**(v, ω) + **불확실성**(covariance) | 위치 + 방향만 |
| 발행 방법 | `create_publisher()` + `.publish()` | `TransformBroadcaster()` + `.sendTransform()` |
| 현재 주로 쓰는 쪽 | Foxglove (화면 표시) | slam_toolbox (스캔 매칭 시작점) |

둘 다 특정 노드 전용 채널이 아니라 ROS 네트워크에 공개로 발행하는 것이고, 지금 시스템에서 누가 주로 가져다 쓰는지가 저렇게 나뉠 뿐이다. slam_toolbox가 실제로 의존하는 건 거의 TF 쪽이고, `/odom` 토픽의 속도·covariance는 핵심 동작엔 거의 안 쓰인다.

**발행자(Publisher) / 구독(Subscription)** 비유: 발행자는 유튜버(채널에 영상을 올림, 누가 보는지 몰라도 올림), 구독은 구독 버튼(새 메시지 오면 콜백 함수가 자동 실행). 한 노드가 어떤 토픽엔 구독자, 어떤 토픽엔 발행자 역할을 동시에 할 수 있다.

---

## 7. slam_toolbox가 TF를 읽어서 하는 일

```text
① /scan 수신
② TF 조회: "odom 기준 지금 base_footprint 어디 있냐"
③ 그 위치를 초기 추측값으로 삼음 (없으면 지도 전체를 뒤져야 해서 느리고 부정확)
④ 그 주변에서 기존 지도와 가장 잘 맞는 위치를 탐색 (스캔 매칭)
⑤ 찾은 진짜 위치 ≠ 추측값 → 차이를 TF(map→odom)로 발행
⑥ 새 스캔을 지도에 반영 → /map 갱신
```

```text
진짜 위치(map 기준) = [map→odom 보정량] + [odom→base_footprint 바퀴 위치]
```

odom 값 자체는 고치지 않고, 얼마나 틀렸는지를 `map→odom`에 누적한다. odom은 빠르고 연속적(단기 추정용), SLAM은 느리지만 오차가 쌓이지 않음(장기 보정용) — 서로 약점을 메우는 구조다.

