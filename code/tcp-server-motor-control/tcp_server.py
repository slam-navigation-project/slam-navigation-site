import socket
import RPi.GPIO as GPIO
import motor_control as motor


HOST = "0.0.0.0"
PORT = 5000

GPIO_PIN = 21   # BCM GPIO 21


FRONT, BACK, LEFT, RIGHT = 0, 1, 2, 3
SPEED = 40
freq = 100

## PWM 세팅
motor.init_set_motor(freq)

# ---------------------------------
# TCP 서버 생성
# ---------------------------------
server_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

server_socket.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

server_socket.bind((HOST, PORT))
server_socket.listen(1)

print("TCP 서버 대기 중...")

running = True


try:

    while running:

        conn, addr = server_socket.accept()

        print("PC 연결됨 :", addr)

        try:

            # 클라이언트가 \n을 붙여 보내므로
            # 한 줄씩 명령을 읽음
            with conn.makefile(
                "r",
                encoding="utf-8"
            ) as client:

                for data in client:

                    command = data.strip().upper()

                    print("수신 :", command)


                    ## 직진
                    if command == "FRONT":
                        motor.setMotor_driving(SPEED, FRONT)
                        print("직진")

                    ## 후진
                    elif command == "BACK":
                        motor.setMotor_driving(SPEED, BACK)
                        print("후진")

                    ## 좌회전
                    elif command == "LEFT":
                        motor.setMotor_driving(SPEED, LEFT)
                        print("좌회전")

                    ## 우회전
                    elif command == "RIGHT":
                        motor.setMotor_driving(SPEED, RIGHT)
                        print("우회전")

                    # -------------------------
                    # 키를 뗀 경우
                    # -------------------------
                    elif command in (
                        "FRONT_STOP",
                        "BACK_STOP",
                        "LEFT_STOP",
                        "RIGHT_STOP"
                    ):

                        motor.setMotor_driving(0, motor.STOP_DRIVE)
                        print("정지")


                    # -------------------------
                    # 프로그램 종료
                    # -------------------------
                    elif command == "Q":

                        print("연결 종료")

                        motor.setMotor_driving(0, motor.STOP_DRIVE)
                        running = False

                        break

        finally:
            # 연결이 갑자기 끊겨도 모터 정지
            motor.setMotor_driving(0, motor.STOP_DRIVE)
            conn.close()

            print("PC 연결 해제")
            conn.close()


except KeyboardInterrupt:

    print("\n서버 종료")


finally:
    # 모터 정지
    motor.setMotor_driving(0, motor.STOP_DRIVE)

    # PWM 종료
    if motor.pwmA is not None:
        motor.pwmA.stop()

    if motor.pwmB is not None:
        motor.pwmB.stop()

    GPIO.cleanup()

    server_socket.close()

    print("TCP 서버 종료 완료")