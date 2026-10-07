---
layout: post
title: "TCP 서버를 활용한 Raspberry Pi-PC 간 통신 및 모터 제어"
date: 2026-10-07 17:00:00 +0900
categories: [Capstone, RaspberryPi, TCP]
tags: [RaspberryPi, TCP, Socket, Python, GPIO, PWM, Motor, AMR]
---

## 1. 동작 구조

PC의 키보드 입력을 Raspberry Pi로 전달하기 위해 **TCP 통신**을 이용하였다.  
PC에서 방향키를 입력하면 해당 입력값을 TCP를 통해 Raspberry Pi로 전송하고, Raspberry Pi는 전달받은 명령에 따라 모터의 회전 방향과 출력을 결정한다.

전체적인 동작 구조는 다음과 같다.

```text
[PC]
키보드 입력
   ↓
tcp_client.py
   ↓
TCP 통신으로 데이터 전송
   ↓
tcp_server.py
   ↓
motor_control.py
   ↓
GPIO / PWM
   ↓
모터 드라이버
   ↓
DC Motor
```

TCP 통신에서는 **서버(Server)**와 **클라이언트(Client)**의 역할이 구분된다.
본 프로젝트에서는 Raspberry Pi를 TCP 서버로, PC를 TCP 클라이언트로 구성하였다.

먼저 Raspberry Pi에서 서버 프로그램을 실행하면 지정된 포트 번호를 열고 클라이언트의 접속을 기다린다.

이후 PC에서 클라이언트 프로그램을 실행하여 Raspberry Pi의 IP 주소와 서버에서 설정한 포트 번호를 이용해 접속한다.

연결이 완료되면 PC에서 발생한 키보드 입력을 문자열 형태의 명령으로 서버에 전송한다.

예를 들어 위쪽 방향키를 누르면 다음과 같은 데이터가 전송된다.

```text
FRONT
```

Raspberry Pi의 TCP 서버는 해당 명령을 수신한 뒤 모터 제어 함수를 호출한다.

통신 및 주행을 종료할 경우 클라이언트에서 종료 명령인 `Q`를 전송한다.  
서버는 이를 수신하면 모터를 정지시킨 후 클라이언트와의 연결 및 서버 소켓을 종료한다.

---

## 2. 코드 구성

PC와 Raspberry Pi에서 실행되는 코드는 다음과 같이 구성하였다.

```text
PC
└── tcp_client.py

Raspberry Pi
├── tcp_server.py
└── motor_control.py
```

각 파일의 역할은 다음과 같다.

| 파일 | 실행 환경 | 역할 |
| --- | --- | --- |
| `tcp_client.py` | PC | 키보드 입력 감지 및 TCP 명령 전송 |
| `tcp_server.py` | Raspberry Pi | TCP 서버 실행 및 수신 명령 처리 |
| `motor_control.py` | Raspberry Pi | GPIO 및 PWM을 이용한 모터 제어 |

---

## 3. 코드 설명

### (1) `tcp_client.py`

`tcp_client.py`는 Raspberry Pi에 데이터를 전송하는 PC에서 실행하는 코드이다.

먼저 Raspberry Pi에서 TCP 서버를 실행한 후, Raspberry Pi의 IP 주소와 서버의 포트 번호를 이용하여 연결한다.

```Python
RASPI_IP = "Raspberry Pi IP"
PORT = 5000

client_socket.connect(
    (RASPI_IP, PORT)
)
```

서버와 클라이언트는 동일한 포트 번호를 사용해야 하며, 본 프로젝트에서는 `5000`번 포트를 사용하였다.

클라이언트는 `pynput` 라이브러리를 이용하여 PC의 키보드 입력을 감지하고, 입력에 대응하는 문자열 명령을 Raspberry Pi로 전송한다.

#### `send_command(command)`

서버에 명령 데이터를 전송하는 함수이다.

```Python
def send_command(command):
    client_socket.sendall(
        (command + "\n").encode("utf-8")
    )
```

전송하려는 문자열 뒤에 `\n`을 추가하여 하나의 명령이 끝났음을 구분할 수 있도록 하였다.

또한 TCP 통신에서는 문자열을 직접 전송할 수 없기 때문에 `encode("utf-8")`을 이용하여 문자열을 바이트 데이터로 변환한 후 전송한다.

---

#### `on_press(key)`

키보드의 방향키가 눌렸을 때 실행되는 함수이다.

예를 들어 위쪽 방향키를 입력한 경우 다음과 같이 `FRONT` 명령을 서버로 전송한다.

```Python
if key == keyboard.Key.up:

    if not up_press:
        up_press = True

        send_command("FRONT")

        print("↑ DOWN")
```

`up_press` 변수를 이용하여 키가 계속 눌려 있을 때 동일한 명령이 반복적으로 전송되는 것을 방지하였다.

각 방향키에 따라 전송되는 명령은 다음과 같다.

| 키 입력 | 전송 명령 | 동작 |
| --- | --- | --- |
| `↑` | `FRONT` | 직진 |
| `↓` | `BACK` | 후진 |
| `←` | `LEFT` | 좌회전 |
| `→` | `RIGHT` | 우회전 |

---

#### `on_release(key)`

방향키에서 손을 뗐을 때 실행되는 함수이다.

예를 들어 위쪽 방향키에서 손을 뗀 경우 다음과 같이 `FRONT_STOP` 명령을 전송한다.

```Python
elif key == keyboard.Key.up:

    up_press = False

    send_command("FRONT_STOP")

    print("↑ UP")
```

이에 따라 키를 누르고 있는 동안에만 차량이 이동하도록 구현하였다.

각 키의 Release 명령은 다음과 같다.

| 키 Release | 전송 명령 |
| --- | --- |
| `↑` | `FRONT_STOP` |
| `↓` | `BACK_STOP` |
| `←` | `LEFT_STOP` |
| `→` | `RIGHT_STOP` |

`ESC` 키를 입력하면 서버에 `Q`를 전송하고 클라이언트 프로그램을 종료한다.

```Python
if key == keyboard.Key.esc:

    send_command("Q")

    print("프로그램 종료")

    return False
```

---

### (2) `tcp_server.py`

`tcp_server.py`는 Raspberry Pi에서 실행되며 TCP 서버를 생성하여 PC로부터 전달되는 데이터를 수신하는 코드이다.

서버는 다음과 같이 Raspberry Pi의 모든 네트워크 인터페이스에서 `5000`번 포트를 통해 클라이언트의 접속을 기다린다.

```Python
HOST = "0.0.0.0"
PORT = 5000

server_socket.bind(
    (HOST, PORT)
)

server_socket.listen(1)
```

`accept()`를 통해 PC의 연결을 기다리며 클라이언트가 접속하면 TCP 통신을 시작한다.

```Python
conn, addr = server_socket.accept()

print("PC 연결됨 :", addr)
```

클라이언트에서는 각각의 명령 뒤에 `\n`을 추가하여 데이터를 전송하기 때문에 서버에서는 줄 단위로 데이터를 읽는다.

```Python
with conn.makefile(
    "r",
    encoding="utf-8"
) as client:

    for data in client:

        command = data.strip().upper()

        print("수신 :", command)
```

수신된 문자열에 따라 `motor_control.py`의 `setMotor_driving()` 함수를 호출하여 모터를 제어한다.

예를 들어 `FRONT` 명령을 수신한 경우 다음과 같이 직진 명령을 전달한다.

```Python
if command == "FRONT":

    motor.setMotor_driving(
        SPEED,
        motor.FRONT
    )
```

각 명령에 따른 동작은 다음과 같다.

```text
FRONT       → 직진
BACK        → 후진
LEFT        → 좌회전
RIGHT       → 우회전

FRONT_STOP  ┐
BACK_STOP   │
LEFT_STOP   ├→ 모터 정지
RIGHT_STOP  ┘

Q           → 모터 정지 및 TCP 통신 종료
```

클라이언트의 연결이 갑자기 종료되는 경우에도 차량이 계속 움직이는 것을 방지하기 위해 연결 종료 시 모터를 정지하도록 구성하였다.

```Python
finally:

    motor.setMotor_driving(
        0,
        motor.STOP_DRIVE
    )

    conn.close()
```

---

### (3) `motor_control.py`

`motor_control.py`는 Raspberry Pi의 GPIO 및 PWM을 이용하여 실제 모터를 구동하는 기능을 담당한다.

`ENA`, `ENB`에는 PWM 신호를 인가하여 각 모터의 속도를 제어하고, `IN1 ~ IN4`를 통해 모터의 회전 방향을 결정한다.

---

#### `init_set_motor(freq)`

모터를 사용하기 전에 GPIO 핀과 PWM 객체를 초기화하는 함수이다.

```Python
def init_set_motor(freq):
    global pwmA, pwmB

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    pwmA = setPinConfig(
        ENA,
        IN1,
        IN2,
        freq
    )

    pwmB = setPinConfig(
        ENB,
        IN3,
        IN4,
        freq
    )
```

TCP 서버가 시작될 때 해당 함수를 한 번 호출하여 모터 제어에 필요한 GPIO 및 PWM을 초기화한다.

---

#### `setMotor_driving(speed, driving_mode)`

차량의 이동 방향과 속도를 전달받아 좌·우 모터의 동작을 결정하는 함수이다.

```Python
def setMotor_driving(speed, driving_mode):

    if driving_mode == FRONT:

        setMotor(
            LEFT_CH,
            speed,
            FORWARD
        )

        setMotor(
            RIGHT_CH,
            speed,
            FORWARD
        )

    elif driving_mode == BACK:

        setMotor(
            LEFT_CH,
            speed,
            BACKWORD
        )

        setMotor(
            RIGHT_CH,
            speed,
            BACKWORD
        )
```

각 주행 방향에 따라 두 모터의 상태를 다르게 설정한다.

이를 통해 TCP 서버에서는 GPIO를 직접 제어하지 않고 다음과 같이 간단하게 차량의 이동 방향을 제어할 수 있다.

```Python
motor.setMotor_driving(
    SPEED,
    motor.FRONT
)
```

따라서 TCP 통신 기능과 실제 모터 제어 기능을 서로 다른 파일로 분리하여 코드의 역할을 명확하게 구성하였다.

---

## 4. 작동 방식

전체 시스템의 실제 동작 과정은 다음과 같다.

```text
① Raspberry Pi에서 tcp_server.py 실행
                ↓
② TCP 서버가 클라이언트의 접속 대기
                ↓
③ PC에서 tcp_client.py 실행
                ↓
④ Raspberry Pi의 IP와 PORT를 통해 TCP 연결
                ↓
⑤ PC에서 방향키 입력
                ↓
⑥ 입력에 대응하는 문자열 명령 전송
                ↓
⑦ Raspberry Pi에서 명령 수신
                ↓
⑧ motor_control.py의 모터 제어 함수 호출
                ↓
⑨ GPIO 및 PWM 출력
                ↓
⑩ AMR 이동
```

예를 들어 위쪽 방향키를 누르고 떼는 경우의 데이터 흐름은 다음과 같다.

```text
[↑ DOWN]

PC
 ↓
tcp_client.py
 ↓
"FRONT"
 ↓
TCP
 ↓
tcp_server.py
 ↓
setMotor_driving(SPEED, FRONT)
 ↓
양쪽 모터 구동
 ↓
직진


[↑ UP]

PC
 ↓
tcp_client.py
 ↓
"FRONT_STOP"
 ↓
TCP
 ↓
tcp_server.py
 ↓
setMotor_driving(0, STOP_DRIVE)
 ↓
양쪽 모터 정지
```

---
### 다이어그램
![다이어그램]
