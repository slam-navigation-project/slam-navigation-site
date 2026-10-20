import socket
from pynput import keyboard


RASPI_IP = "100.88.126.54"    # Raspberry Pi IP
PORT = 5000


client_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

client_socket.connect(
    (RASPI_IP, PORT)
)


print("Raspberry Pi 연결 완료")
print("↑ 화살표를 누르는 동안 직진")
print("↓ 화살표를 누르는 동안 후진")
print("←, → 화살표를 누르는 동안 차체 회전")
print("ESC : 종료")

# 현재 화살표가 눌려있는지 기록
up_press, down_press, left_press, right_press = False, False, False, False


def send_command(command):
    client_socket.sendall(
        (command + "\n").encode("utf-8")
    )


# 키를 누를 때
def on_press(key):
    global up_press, down_press, left_press, right_press

    if key == keyboard.Key.up:

        # 키보드 자동 반복으로 Y가 계속 전송되는 것 방지
        if not up_press:
            up_press = True
            send_command("FRONT")
            print("↑ DOWN")

    elif key == keyboard.Key.down:

        # 키보드 자동 반복으로 Y가 계속 전송되는 것 방지
        if not down_press:
            down_press = True
            send_command("BACK")
            print("↓ DOWN")

    if key == keyboard.Key.left:

        # 키보드 자동 반복으로 Y가 계속 전송되는 것 방지
        if not left_press:
            left_press = True
            send_command("LEFT")
            print("← DOWN")

    elif key == keyboard.Key.right:

        # 키보드 자동 반복으로 Y가 계속 전송되는 것 방지
        if not right_press:
            right_press = True
            send_command("RIGHT")
            print("→ DOWN")

        


# 키를 뗄 때
def on_release(key):
    global up_press, down_press, left_press, right_press

    if key == keyboard.Key.esc:

        send_command("Q")

        print("프로그램 종료")

        return False

    elif key == keyboard.Key.up:

        up_press = False
        send_command("FRONT_STOP")
        print("↑ UP")

    elif key == keyboard.Key.down:

        down_press = False
        send_command("BACK_STOP")
        print("↓ UP")

    elif key == keyboard.Key.left:

        left_press = False
        send_command("LEFT_STOP")
        print("← UP")

    elif key == keyboard.Key.right:

        right_press = False
        send_command("RIGHT_STOP")
        print("→ UP")


with keyboard.Listener(
    on_press=on_press,
    on_release=on_release
) as listener:

    listener.join()


client_socket.close()