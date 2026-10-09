---
layout: post
title: "엔코더 기반 바퀴 속도 제어 (피드포워드 + PI)"
date: 2026-10-09 20:00:00 +0900
categories: [Capstone, RaspberryPi, AMR]
tags: [RaspberryPi, Encoder, lgpio, PI-Control, Feedforward, Odometry, Python]
---

## 1. 진행 개요

- 엔코더 달린 TT 모터(DFRobot FIT0450)로 교체 후, 바퀴가 실제로 얼마나 돌았는지 측정.
- 같은 PWM을 줘도 좌우 바퀴 속도가 다른 문제 확인.
- 모터 특성 측정(캘리브레이션) + 피드포워드 + PI 제어로 **두 바퀴가 목표 각속도를 정확히 내도록** 구성.
- 엔코더 카운트로 차체 위치(x, y, θ)를 계산하는 오도메트리 구현.

> 이 글은 오늘 작업한 내용 중 **엔코더** 부분이다.  
> 자이로 센서는 [MPU6050 자이로로 방향 유지하기]({% post_url 2026-10-09-mpu6050-gyro-heading %}),  
> ROS2/SLAM 연동은 [바퀴 오도메트리 기반 SLAM]({% post_url 2026-10-09-wheel-odometry-slam %})에 정리하였다.

---

## 2. 하드웨어 구성

| 항목 | 내용 |
| --- | --- |
| 모터 | DFRobot FIT0450 (6V, 감속비 120:1, 엔코더 8 PPR) |
| 모터 드라이버 전원 | 라즈베리파이 5V (9V 배터리 분리도 시도, [2편 7장]({% post_url 2026-10-09-mpu6050-gyro-heading %}) 참고) |
| 제어 | Raspberry Pi, `gpiozero`(PWM) + `lgpio`(엔코더 인터럽트) |

| 신호 | 왼쪽 | 오른쪽 |
| --- | --- | --- |
| 모터 방향 | GPIO 23, 24 | GPIO 27, 22 |
| PWM (Enable) | GPIO 12 | GPIO 13 |
| 엔코더 A / B | GPIO 5 / 6 | GPIO 19 / 26 |

GPIO 12, 13은 라즈베리파이의 **하드웨어 PWM** 핀이라 모터 Enable에 사용하였다.  
엔코더는 아무 GPIO나 입력으로 쓸 수 있어 남는 핀(5, 6, 19, 26)에 연결하였다.

---

## 3. 코드 구성 (`tracker_encoder.py`)

하나의 파일에 하드웨어 제어를 모두 넣고, ROS2 노드에서는 이 파일을 `import`해서 사용한다.

```text
tracker_encoder.py
├── Encoder              엔코더 카운트 (인터럽트)
├── MPU6050              자이로 z축 각속도          → 2편
├── WheelSpeedController 바퀴 1개의 속도 제어 (피드포워드 + PI)
├── Robot                제어 루프 스레드 + 오도메트리 + 방향 유지
├── main()               단독 실행 시 키보드 조종 화면 (curses)
└── calibrate()          --calibrate 옵션: 모터 특성 측정 → motor_calib.json
```

```bash
python3 ~/tracker_encoder.py               # 단독 키보드 주행 (W/A/S/D)
python3 ~/tracker_encoder.py --calibrate   # 모터 특성 측정 (바퀴 띄우고)
```

---

## 4. 엔코더 읽기

### (1) 쿼드러처 엔코더 원리

엔코더는 A상, B상 두 개의 펄스를 출력하며 두 신호는 90° 위상차를 가진다.  
A상이 바뀌는 순간 B상의 레벨을 보면 회전 방향을 알 수 있다.

```text
정방향:  A ┌─┐_┌─┐_      A 상승 시 B = 0
         B _┌─┐_┌─┐      A 하강 시 B = 1
```

### (2) 코드

```python
class Encoder:
    def __init__(self, chip, pin_a, pin_b, invert=False):
        ...
        lgpio.gpio_claim_input(chip, pin_b, PULL_UP)
        lgpio.gpio_claim_alert(chip, pin_a, lgpio.BOTH_EDGES, PULL_UP)
        self._cb = lgpio.callback(chip, pin_a, lgpio.BOTH_EDGES, self._on_edge)

    def _on_edge(self, chip, gpio, level, tick):
        if level > 1:          # 엣지가 아닌 워치독 이벤트
            return
        b = lgpio.gpio_read(self.chip, self.pin_b)
        self.count += self.sign if level != b else -self.sign
```

**왜 이렇게 했나**

- **인터럽트(callback) 방식**: 반복문으로 핀을 계속 읽으면(polling) 펄스를 놓친다. `lgpio`의 엣지 알림을 쓰면 커널이 엣지 시점을 잡아 주므로 놓치지 않는다.
- **2체배 (A상 양쪽 엣지만)**: A, B 양쪽 엣지를 모두 세는 4체배도 가능하지만, 파이썬 콜백은 호출마다 비용이 커서 CPU 부담이 두 배가 된다. 2체배로도 해상도가 충분하다.

```text
바퀴 1회전 = 8 (PPR) × 120 (감속비) × 2 (엣지) = 1920 카운트
1카운트 ≈ 0.16 mm 이동 (바퀴 반지름 약 4.8 cm 기준)
```

- **`invert`**: 좌우 모터가 서로 마주 보게 장착되어 있어, 둘 다 전진해도 한쪽은 카운트가 줄어든다. 오른쪽만 부호를 뒤집었다 (`ENC_R_INVERT = True`).
- **`PULL_UP` getattr**: `lgpio` 버전에 따라 상수 이름이 `SET_PULL_UP` / `SET_BIAS_PULL_UP`로 달라서 `AttributeError`가 났다. 둘 다 시도하도록 처리하였다.

```python
PULL_UP = getattr(lgpio, "SET_BIAS_PULL_UP", None) or getattr(lgpio, "SET_PULL_UP")
```

---

## 5. 문제: 같은 PWM인데 속도가 다르다

같은 PWM 값을 두 모터에 주어도 한쪽 바퀴가 느리게 돌아 차가 한쪽으로 휘었다.  
실제 측정 결과(`motor_calib.json`)를 보면 모터마다 특성이 꽤 다르다.

| 구분 | 돌기 시작하는 PWM | PWM 0.20일 때 각속도 |
| --- | --- | --- |
| 왼쪽 전진 | 0.08 → 0.10 | 3.65 rad/s |
| 왼쪽 후진 | 0.08 → 0.10 | 3.67 rad/s |
| 오른쪽 전진 | 0.06 → 0.08 | 3.32 rad/s |
| 오른쪽 후진 | **0.12 → 0.14** | 3.39 rad/s |

- 모터마다 **돌기 시작하는 PWM(데드존)** 이 다르다. 오른쪽 후진은 0.12까지 아예 안 돈다.
- 같은 PWM 0.20에서도 왼쪽이 약 10% 빠르다.
- 같은 모터라도 **전진/후진 특성이 다르다.**

처음에는 `ω = k × (PWM − dead)` 직선 하나로 근사하였으나, 저속 구간이 직선이 아니어서  
데드존이 0.688처럼 말이 안 되는 값으로 계산되었다. 그래서 **측정값 표 + 직선 보간**으로 바꾸었다.

---

## 6. 모터 캘리브레이션 (`calibrate()`)

바퀴를 바닥에서 띄운 상태에서 실행한다.

```bash
python3 ~/tracker_encoder.py --calibrate
```

```python
for u in CALIB_PWMS:                      # 0.06 ~ 0.60, 낮은 구간은 촘촘히
    drive(motor_left, sign * u)
    drive(motor_right, sign * u)
    time.sleep(0.7)                       # 속도가 안정될 때까지 대기
    c0 = {w: e.count for w, e in enc.items()}
    t0 = time.monotonic()
    time.sleep(0.8)                       # 0.8초 동안 카운트 측정
    ...
    omega = (e.count - c0[w]) * RAD_PER_COUNT / (t1 - t0) * sign
```

**왜 이렇게 했나**

- **바퀴를 띄우는 이유**: 바닥에 두면 차가 움직이면서 마찰, 무게가 섞여 모터 자체 특성이 아닌 값이 나온다. 띄운 상태 값을 기준으로 잡고, 실제 주행 중 차이는 PI가 보정한다.
- **0.7초 대기 후 측정**: PWM을 바꾼 직후에는 가속 중이라 속도가 낮게 측정된다.
- **저속 구간을 촘촘하게**: 실제 주행 속도(1~3 rad/s)가 PWM 0.1~0.2 구간이고, 이 구간이 가장 비선형이다.
- **표 정리 규칙**: 돌기 시작한 바로 앞 PWM을 "속도 0" 점으로 넣고, 속도가 오히려 줄거나 안 느는 점(포화, 측정 튐)은 버려서 표가 항상 증가하도록 하였다. 그래야 역으로 찾을 때 답이 하나로 정해진다.

```json
"R": { "fwd": [[0.06, 0.0], [0.08, 0.462], [0.10, 1.034], [0.12, 1.528], ...],
       "bwd": [[0.12, 0.0], [0.14, 1.88],  [0.16, 2.515], ...] }
```

---

## 7. 속도 제어: 피드포워드 + PI (`WheelSpeedController`)

### (1) 구조

```text
목표 각속도 ω*
   ├─▶ 피드포워드: 표에서 "ω*를 내려면 필요한 PWM"을 역으로 찾음  (대부분을 담당)
   └─▶ PI: (ω* − 측정 ω) 오차를 보정                               (남은 차이만)
         ↓
      PWM = 피드포워드 + Kp·오차 + ∫Ki·오차
```

### (2) 피드포워드

```python
def feedforward(self, target):
    table = self.calib["fwd" if target > 0 else "bwd"]   # [[PWM, ω], ...]
    w = abs(target)
    for (u0, w0), (u1, w1) in zip(table, table[1:]):
        if w <= w1:
            u = u0 + (u1 - u0) * (w - w0) / (w1 - w0)    # 두 측정점 사이 직선 보간
            return math.copysign(u, target)
    # 표 범위를 넘으면 마지막 구간 기울기로 연장
```

같은 목표 2.0 rad/s에 대해 실제 표에서 계산되는 PWM은 다음과 같다.

| | 전진 | 후진 |
| --- | --- | --- |
| 왼쪽 | 0.130 | 0.133 |
| 오른쪽 | 0.138 | 0.144 |

**목표 각속도는 같지만 나가는 PWM은 바퀴마다 다르다.** 이것이 "같은 속도로 돌게 하는" 핵심이다.

**왜 PI만 쓰지 않고 피드포워드를 넣었나**  
PI만 있으면 정지 상태에서 출발할 때 적분이 데드존(0.08~0.12)을 넘을 만큼 쌓일 때까지 바퀴가 안 돈다.  
그 사이 한쪽이 먼저 출발하면 차가 틀어진다. 피드포워드가 처음부터 거의 맞는 PWM을 주므로 **양쪽이 동시에 출발**한다.

### (3) PI

```python
def update(self, target, measured, dt):
    if target == 0.0:
        self.reset(); self._last_sign = 0
        return 0.0
    sign = 1 if target > 0 else -1
    if sign != self._last_sign:      # 방향이 바뀌면 이전 방향에서 쌓인 적분값 버림
        self.reset()
        self._last_sign = sign
    err = target - measured
    self.integ += KI * err * dt
    self.integ = max(-I_LIMIT, min(I_LIMIT, self.integ))   # 적분 상한
    u = self.feedforward(target) + KP * err + self.integ
    return max(-1.0, min(1.0, u))
```

**왜 이렇게 했나**

- **D(미분)를 뺀 이유**: 20ms 동안 들어오는 카운트는 정수(수십 개)라서 측정 각속도가 계단처럼 튄다. 미분은 이 튐을 그대로 키워 모터가 떨린다. 대신 측정값에 저역통과 필터(`OMEGA_FILTER = 0.5`)를 걸었다.
- **적분 상한 (`I_LIMIT`)**: 바퀴가 걸려서 못 돌 때 적분이 무한히 쌓이면, 풀리는 순간 확 튀어 나간다 (적분 와인드업).
- **방향이 바뀌면 적분 초기화**: 전진하며 쌓인 +적분이 후진 시작 때 남아 있으면 처음 잠깐 반대로 힘을 준다. "전후진 바꿀 때 바퀴가 확 돈다"는 문제의 원인 중 하나였다.

### (4) 가속 제한 (`_step`)

```python
step = ACCEL_LIMIT * dt
for i, goal in enumerate(self.target):
    diff = goal - self.cmd[i]
    self.cmd[i] += max(-step, min(step, diff))
```

목표를 한 번에 바꾸지 않고 `ACCEL_LIMIT`(3.0 rad/s²) 기울기로 서서히 따라가게 하였다.  
급출발, 급정지 때 바퀴가 미끄러지면 **엔코더는 돌았는데 차는 안 간** 상태가 되어 오도메트리가 틀어지기 때문이다 (처음 6.0 → 3.0으로 낮춤).

### (5) 좌우 동기화 (제자리 회전)

```python
sl, sr = math.copysign(1, self.target[0]), math.copysign(1, self.target[1])
prog_l = (cl - self._seg_start[0]) * RAD_PER_COUNT * sl
prog_r = (cr - self._seg_start[1]) * RAD_PER_COUNT * sr
sync = K_SYNC * (prog_l - prog_r)
tl_adj = tl - sl * sync
tr_adj = tr + sr * sync
```

두 바퀴가 같은 크기로 돌아야 하는 명령(제자리 회전 등)에서는, 명령 시작 이후 **누적 회전량**을 비교해 앞선 쪽을 늦추고 뒤처진 쪽을 당긴다.  
속도만 맞추면 순간순간의 작은 차이가 쌓이는데, 누적량을 보면 그 쌓인 차이까지 되돌릴 수 있다.  
직진일 때는 이 대신 자이로 방향 유지를 쓴다 (2편).

### (6) 주요 파라미터

| 파라미터 | 값 | 의미 |
| --- | --- | --- |
| `CONTROL_DT` | 0.02 s | 제어 주기 (50Hz) |
| `KP`, `KI` | 0.04, 0.15 | PI 게인 |
| `I_LIMIT` | 0.5 | 적분항 상한 |
| `K_SYNC` | 2.0 | 좌우 동기화 세기 |
| `ACCEL_LIMIT` | 3.0 rad/s² | 가속 제한 |
| `MAX_WHEEL_OMEGA` | 6.0 rad/s | 바퀴 속도 상한 (측정 최대값 근처) |

---

## 8. 오도메트리 계산

### (1) 코드

```python
cl, cr = self.enc_l.count, self.enc_r.count
dcl, dcr = cl - prev_l, cr - prev_r                 # 20ms 동안 늘어난 카운트
dl, dr = dcl * M_PER_COUNT, dcr * M_PER_COUNT        # 각 바퀴가 실제로 굴러간 거리
d = (dl + dr) / 2.0                                  # 차체 중심 이동 거리
dth = self._gyro_dtheta(dt, dl, dr)                  # 회전각 (자이로, 끊기면 엔코더)
self.x += d * math.cos(self.theta + dth / 2.0)
self.y += d * math.sin(self.theta + dth / 2.0)
self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))
```

### (2) 각속도가 아니라 "돈 양(카운트)"으로 계산한다

위치 계산에는 **각속도를 쓰지 않는다.**  
엔코더 카운트는 바퀴가 실제로 돈 각도를 하나도 빠짐없이 센 값이므로, 20ms 안에서 빨랐다 느렸다 해도 그 사이에 **돈 총량만** 쌓인다.  
따라서 바퀴 속도가 일정하지 않은 것은 위치 계산에 거의 영향을 주지 않는다.

- 각속도(`omega_l`, `omega_r`)는 **속도 제어**와 `/odom`의 속도 표시에만 쓴다.
- `θ + dθ/2`: 20ms 동안 방향이 조금 바뀌었으므로 **중간 방향**으로 이동했다고 계산하여 곡선 주행 오차를 줄였다.
- `atan2(sin, cos)`: θ를 −180° ~ 180° 범위로 유지한다.
- 회전각은 바퀴 차이 `(dr − dl) / L` 대신 **자이로**를 쓴다. 바퀴 차이 방식은 미끄러짐과 바퀴 간격 측정 오차에 매우 약하기 때문이다 (2편).

### (3) 그럼 오차는 어디서 생기나

속도 변화가 아니라 **"바퀴는 돌았는데 차는 그만큼 안 간" 경우**가 오차를 만든다.

| 원인 | 영향 |
| --- | --- |
| 바퀴 미끄러짐 | 급출발/급정지/제자리 회전 시 헛돌면 카운트만 늘어 거리가 크게 계산됨 → `ACCEL_LIMIT`로 완화 |
| 바퀴 반지름 오차 | 카운트 → 거리 배율이 틀리면 이동할수록 일정 비율로 오차 누적 |
| 차체 기울어짐, 접지 불량 | 바퀴가 바닥을 제대로 누르지 못해 미끄러짐 증가 |
| 고무 바퀴 눌림 | 하중이 크면 실제 굴림 반지름이 작아짐 → 반지름 보정으로 흡수 |

### (4) 바퀴 반지름 보정

공칭 반지름(33mm) 대신 실제 주행 거리로 보정하였다.  
줄자를 깔고 주행한 뒤 `실제 거리 / 화면 거리`를 곱한다.

```python
WHEEL_RADIUS = 0.033 * (65 / 45)   # 지금 값 × 실제 거리 / 화면 거리
TRACK_WIDTH = 0.12                 # 좌우 바퀴 접지면 중심 사이 거리
```

---

## 9. 결과

- 좌우 모터에 서로 다른 PWM이 나가면서 **측정 각속도가 목표대로 맞춰지는 것** 확인.
- 전후진 전환 시 바퀴가 튀는 현상 완화 (적분 초기화 + 가속 제한).
- 엔코더 기반 x, y 위치 계산 확인.
- 다만 차체 무게 때문에 엔코더만으로는 직진 시 조금씩 틀어짐 → [자이로 센서 추가]({% post_url 2026-10-09-mpu6050-gyro-heading %}).
