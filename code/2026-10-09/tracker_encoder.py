import curses
import json
import math
import os
import sys
import threading
import time

import lgpio
from gpiozero import Motor

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# ==========================================
# 1. 물리 파라미터 (단위: m, rad)
# ==========================================
WHEEL_RADIUS = 0.033 * (65 / 45)     #지금 값(0.033) × 실제 거리 / 화면 거리
TRACK_WIDTH = 0.12        # 좌우 바퀴 간격 L (m), 바퀴 접지면 중심 사이를 측정

# DFRobot FIT0450: 모터축 1회전당 A상 8펄스, 기어비 120:1
PPR_MOTOR = 8
GEAR_RATIO = 120
EDGES_PER_PULSE = 2       # A상 상승+하강 엣지 모두 카운트 (2체배)
COUNTS_PER_REV = PPR_MOTOR * GEAR_RATIO * EDGES_PER_PULSE   # 바퀴 1회전 = 1920 카운트
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV
M_PER_COUNT = WHEEL_RADIUS * RAD_PER_COUNT

# ==========================================
# 2. 핀 배치 (BCM 번호)
# ==========================================
motor_left = Motor(forward=23, backward=24, enable=12)
motor_right = Motor(forward=27, backward=22, enable=13)

ENC_L_A, ENC_L_B = 5, 6
ENC_R_A, ENC_R_B = 19, 26

# 전진하는데 카운트가 줄어들면 그쪽 값을 반대로 바꾸세요.
# 모터가 서로 마주보게 장착되면 보통 한쪽이 반대입니다.
ENC_L_INVERT = False
ENC_R_INVERT = True

# lgpio 버전에 따라 풀업 상수 이름이 다름
PULL_UP = getattr(lgpio, "SET_BIAS_PULL_UP", None) or getattr(lgpio, "SET_PULL_UP")

GPIO_CHIP = 0             # Pi 5 + 구버전 OS에서 실패하면 4로 변경

# ==========================================
# 3. 속도 제어 파라미터
# ==========================================
TARGET_OMEGA = 0.3 * 2.0 * math.pi  # 목표 바퀴 각속도 (rad/s) = 1초에 0.3바퀴 (18 RPM, 약 6.2cm/s)
CONTROL_DT = 0.02         # 제어 주기 (s), 50Hz

# 피드포워드: 직접 측정한 'PWM 0.3에서 10초 5바퀴' = 3.14 rad/s -> 0.3 / 3.14 ≈ 0.095
KFF = 0.095               # 보정 파일(motor_calib.json)이 없을 때만 쓰는 기본 피드포워드
MAX_WHEEL_OMEGA = 6.0     # 바퀴 각속도 상한 (rad/s). 모터 보정 측정의 최대값 근처로
ACCEL_LIMIT = 3.0         # 목표 바퀴 각속도 변화 제한 (rad/s²). 0→3 rad/s에 0.5초. 줄일수록 부드러움
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_calib.json")
CALIB_PWMS = [0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.23, 0.26, 0.30, 0.35, 0.40, 0.50, 0.60]  # 낮은 구간을 촘촘히
KP = 0.04                 # 비례 게인 (rad/s 오차당 PWM)
KI = 0.15                 # 적분 게인
I_LIMIT = 0.5             # 적분항 상한 (PWM 단위)
K_SYNC = 2.0              # 좌우 누적 회전량 차이 보정 세기 (rad 차이당 rad/s)
OMEGA_FILTER = 0.5        # 측정 속도 저역통과 필터 (0~1, 클수록 반응 빠름)

# ==========================================
# 3-1. 자이로 (MPU6050) 방향 유지
# ==========================================
USE_GYRO = True           # MPU6050 없이 돌리려면 False
I2C_BUS = 1               # SDA=GPIO2, SCL=GPIO3
MPU_ADDR = 0x68           # i2cdetect -y 1 로 확인 (AD0를 VCC에 연결하면 0x69)
GYRO_Z_SIGN = 1          # 측정 결과 반대로 나와서 -1. A(좌회전) 했을 때 θ가 증가해야 정상

GYRO_SCALE = 1.0 * (360 / 350)         # 스케일 보정: 실제 360° 돌렸을 때 화면이 350°면 360/350 = 1.029

GYRO_DT = 0.005           # 자이로 적분 주기 (s), 200Hz - 제어 루프와 별도로 빠르게 읽음
GYRO_DLPF = 0x04          # 센서 내부 저역통과 필터: 0x03=44Hz, 0x04=21Hz, 0x05=10Hz (진동 많으면 숫자 올리기)
MAX_GYRO_DPS = 300.0      # 이보다 큰 값은 노이즈로 깨진 값으로 보고 버림 (차가 이렇게 빨리 돌 일은 없음)
MAX_GYRO_JUMP_DPS = 150.0 # 5ms 사이에 이만큼 튀면 깨진 값으로 보고 버림
K_HEAD = 2.0              # 방향 오차 보정 세기 (rad 오차당 바퀴 rad/s)
HEAD_CORR_LIMIT = 1.5     # 보정량 상한 (rad/s)


# ==========================================
# 4. 엔코더
# ==========================================
class Encoder:
    """A상 양쪽 엣지에서 인터럽트, B상 레벨로 방향 판별."""

    def __init__(self, chip, pin_a, pin_b, invert=False):
        self.chip = chip
        self.pin_b = pin_b
        self.sign = -1 if invert else 1
        self.count = 0
        lgpio.gpio_claim_input(chip, pin_b, PULL_UP)
        lgpio.gpio_claim_alert(chip, pin_a, lgpio.BOTH_EDGES, PULL_UP)
        self._cb = lgpio.callback(chip, pin_a, lgpio.BOTH_EDGES, self._on_edge)

    def _on_edge(self, chip, gpio, level, tick):
        if level > 1:          # 엣지가 아닌 워치독 이벤트
            return
        b = lgpio.gpio_read(self.chip, self.pin_b)
        # A가 B보다 앞설 때: A 상승 시 B=0, A 하강 시 B=1 -> +1
        self.count += self.sign if level != b else -self.sign

    def close(self):
        self._cb.cancel()


# ==========================================
# 4-1. MPU6050 (z축 각속도만 사용)
# ==========================================
class MPU6050:
    PWR_MGMT_1 = 0x6B
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    GYRO_ZOUT_H = 0x47
    LSB_PER_DPS = 65.5    # ±500 deg/s 범위 (손으로 빨리 돌려도 포화되지 않게)

    def __init__(self, bus=I2C_BUS, addr=MPU_ADDR):
        if SMBus is None:
            raise RuntimeError("smbus2가 없습니다: sudo apt install python3-smbus2")
        self.bus = SMBus(bus)
        self.addr = addr
        self.bias = 0.0
        self.wake()
        time.sleep(0.1)

    def wake(self):
        """초기 설정. 전압 강하로 센서가 리셋되면 슬립 상태로 돌아가므로 다시 호출해야 함."""
        self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0x00)   # 슬립 해제
        self.bus.write_byte_data(self.addr, self.CONFIG, GYRO_DLPF)  # 디지털 저역통과 필터
        self.bus.write_byte_data(self.addr, self.GYRO_CONFIG, 0x08)  # ±500 deg/s

    def _raw_z(self):
        hi, lo = self.bus.read_i2c_block_data(self.addr, self.GYRO_ZOUT_H, 2)
        v = (hi << 8) | lo
        return v - 65536 if v >= 32768 else v

    def calibrate(self, samples=300):
        """정지 상태에서 영점(바이어스) 측정. 이 동안 차를 움직이면 안 됩니다."""
        total = 0
        for _ in range(samples):
            total += self._raw_z()
            time.sleep(0.003)
        self.bias = total / samples

    def rate_z(self):
        """z축 각속도 (rad/s), 반시계 방향 +"""
        dps = (self._raw_z() - self.bias) / self.LSB_PER_DPS
        return GYRO_Z_SIGN * math.radians(dps)

    def close(self):
        self.bus.close()


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


# ==========================================
# 5. 바퀴별 속도 제어기 (피드포워드 + PI)
# ==========================================
class WheelSpeedController:
    """피드포워드(모터별 보정값) + PI"""

    def __init__(self, calib=None):
        self.calib = calib        # {"fwd": {"dead":..,"k":..}, "bwd": {...}} 또는 None
        self.integ = 0.0
        self._last_sign = 0

    def reset(self):
        self.integ = 0.0

    def feedforward(self, target):
        """목표 각속도를 내려면 필요한 PWM (측정한 모터 특성의 역함수)"""
        if target == 0.0:
            return 0.0
        if self.calib is None:
            return KFF * target
        table = self.calib["fwd" if target > 0 else "bwd"]   # [[PWM, ω], ...] ω 오름차순
        w = abs(target)
        for (u0, w0), (u1, w1) in zip(table, table[1:]):
            if w <= w1:
                u = u0 + (u1 - u0) * (w - w0) / (w1 - w0)    # 두 측정점 사이 직선 보간
                return math.copysign(u, target)
        (u0, w0), (u1, w1) = table[-2], table[-1]               # 표 범위를 넘으면 마지막 구간 기울기로 연장
        u = u1 + (u1 - u0) * (w - w1) / (w1 - w0)
        return math.copysign(min(1.0, u), target)

    def update(self, target, measured, dt):
        if target == 0.0:
            self.reset()
            self._last_sign = 0
            return 0.0
        sign = 1 if target > 0 else -1
        if sign != self._last_sign:      # 방향이 바뀌면 이전 방향에서 쌓인 적분값 버림
            self.reset()
            self._last_sign = sign
        err = target - measured
        self.integ += KI * err * dt
        self.integ = max(-I_LIMIT, min(I_LIMIT, self.integ))
        u = self.feedforward(target) + KP * err + self.integ
        return max(-1.0, min(1.0, u))


def load_calib():
    try:
        with open(CALIB_FILE, encoding="utf-8") as f:
            c = json.load(f)
        if not isinstance(c["L"]["fwd"], list):   # 예전 형식(dead/k) 파일이면 무시하고 다시 측정 필요
            return None
        return c
    except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError):
        return None


def drive(motor, u):
    if u > 0.0:
        motor.forward(u)
    elif u < 0.0:
        motor.backward(-u)
    else:
        motor.stop()


# ==========================================
# 6. 제어 루프 + 오도메트리 (별도 스레드)
# ==========================================
class Robot:
    def __init__(self):
        self.chip = lgpio.gpiochip_open(GPIO_CHIP)
        self.enc_l = Encoder(self.chip, ENC_L_A, ENC_L_B, ENC_L_INVERT)
        self.enc_r = Encoder(self.chip, ENC_R_A, ENC_R_B, ENC_R_INVERT)
        self.calib = load_calib()
        self.ctl_l = WheelSpeedController(self.calib["L"] if self.calib else None)
        self.ctl_r = WheelSpeedController(self.calib["R"] if self.calib else None)
        self.cmd = [0.0, 0.0]             # 가속 제한을 거친 실제 목표 (왼쪽, 오른쪽)

        self.imu = None
        if USE_GYRO:
            self.imu = MPU6050()
            self.imu.calibrate()        # 시작할 때 차가 가만히 있어야 함
        self.heading_target = 0.0
        self.hold_heading = False         # True면 자이로로 방향 유지 (직진 명령일 때)
        self._gyro_yaw = 0.0              # 자이로 스레드가 적분한 누적 각도 (rad)
        self._prev_gyro_yaw = 0.0
        self._gyro_last_ok = time.monotonic()
        self.gyro_dps = 0.0               # 현재 z축 각속도 (deg/s, 화면 표시용)
        self.gyro_spikes = 0              # 튀는 값으로 버린 횟수
        self.gyro_reads = 0               # 자이로 읽기 시도 횟수 (실패율 계산용)
        self.i2c_errors = 0               # 누적 I2C 에러 횟수 (화면 표시용)
        self.last_error = ""

        self.state = "STOP"
        self.target = (0.0, 0.0)          # (왼쪽, 오른쪽) 목표 rad/s
        self._seg_start = (0, 0)          # 현재 명령 시작 시점 카운트 (동기화용)

        # 측정값 / 표시용
        self.omega_l = self.omega_r = 0.0
        self.pwm_l = self.pwm_r = 0.0
        self.x = self.y = self.theta = 0.0
        self.distance = 0.0

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._gyro_thread = None
        if self.imu is not None:
            self._gyro_thread = threading.Thread(target=self._gyro_loop, daemon=True)
            self._gyro_thread.start()

    # --- 명령 ---
    def command(self, state):
        w = TARGET_OMEGA
        targets = {
            "FORWARD": (w, w),
            "BACKWARD": (-w, -w),
            "TURN_LEFT": (-w, w),
            "TURN_RIGHT": (w, -w),
            "STOP": (0.0, 0.0),
        }
        self.state = state
        self.heading_target = self.theta
        self.hold_heading = state in ("FORWARD", "BACKWARD")
        self._seg_start = (self.enc_l.count, self.enc_r.count)
        self.target = targets[state]

    def set_velocity(self, v, omega):
        """차체 속도 명령. v: 전진 속도 (m/s), omega: 회전 속도 (rad/s, 반시계 +).
        ROS의 /cmd_vel(Twist)과 같은 형태. 같은 명령이 반복해서 들어와도 됨."""
        wl = (v - omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        wr = (v + omega * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        peak = max(abs(wl), abs(wr))
        if peak > MAX_WHEEL_OMEGA:               # 한쪽이 한계를 넘으면 비율을 유지한 채 둘 다 줄임
            wl *= MAX_WHEEL_OMEGA / peak
            wr *= MAX_WHEEL_OMEGA / peak
        new_target = (wl, wr)
        if new_target == self.target:
            return                               # 같은 명령 반복: 방향 유지 기준/동기화 기준을 초기화하지 않음
        hold = (omega == 0.0 and v != 0.0)
        if hold and not self.hold_heading:
            self.heading_target = self.theta     # 직진이 시작되는 순간의 방향을 유지
        self.hold_heading = hold
        self._seg_start = (self.enc_l.count, self.enc_r.count)
        self.state = "VEL" if (wl or wr) else "STOP"
        self.target = new_target

    # --- 제어 루프 ---
    def _loop(self):
        prev_l, prev_r = self.enc_l.count, self.enc_r.count
        prev_t = time.monotonic()
        while self._running:
            time.sleep(CONTROL_DT)
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            try:
                prev_l, prev_r = self._step(dt, prev_l, prev_r)
            except Exception as e:
                # 어떤 에러가 나도 제어 스레드가 죽지 않게 하고, 일단 정지
                self.last_error = f"{type(e).__name__}: {e}"
                self.target = (0.0, 0.0)
                self.state = "STOP"
                motor_left.stop()
                motor_right.stop()

    def _gyro_loop(self):
        """200Hz로 자이로를 읽어 사다리꼴 적분. 빠른 회전도 놓치지 않게 제어 루프와 분리."""
        prev_rate = 0.0
        prev_t = time.monotonic()
        streak = 0
        while self._running:
            time.sleep(GYRO_DT)
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            self.gyro_reads += 1
            try:
                rate = self.imu.rate_z() * GYRO_SCALE
            except OSError as e:
                self.i2c_errors += 1
                self.last_error = f"I2C: {e}"
                streak += 1
                if streak % 20 == 0:
                    try:
                        self.imu.wake()      # 센서가 리셋됐을 수 있으니 재설정
                    except OSError:
                        pass
                if streak > 3:
                    prev_rate = 0.0
                    continue                 # 길게 끊기면 적분 중단 (제어 루프가 엔코더로 대체)
                rate = prev_rate             # 짧은 끊김은 직전 각속도로 메움
            else:
                streak = 0
                self._gyro_last_ok = now
                dps = math.degrees(rate)
                if abs(dps) > MAX_GYRO_DPS or abs(dps - math.degrees(prev_rate)) > MAX_GYRO_JUMP_DPS:
                    self.gyro_spikes += 1    # 노이즈로 깨진 값 -> 직전 값으로 대체
                    rate = prev_rate
                self.gyro_dps = math.degrees(rate)
            self._gyro_yaw += 0.5 * (rate + prev_rate) * dt
            prev_rate = rate

    def _gyro_dtheta(self, dt, dl, dr):
        """이번 제어 주기의 회전각. 자이로가 끊겼으면 엔코더로 대체."""
        enc_dth = (dr - dl) / TRACK_WIDTH
        if self.imu is None:
            return enc_dth
        yaw = self._gyro_yaw
        d = yaw - self._prev_gyro_yaw
        self._prev_gyro_yaw = yaw
        if time.monotonic() - self._gyro_last_ok > 0.1:
            return enc_dth
        return d

    def _step(self, dt, prev_l, prev_r):
        """제어 1주기. 이번 카운트를 반환 (다음 주기의 prev 값)."""
        cl, cr = self.enc_l.count, self.enc_r.count
        dcl, dcr = cl - prev_l, cr - prev_r

        # 측정 각속도 (저역통과 필터)
        wl = dcl * RAD_PER_COUNT / dt
        wr = dcr * RAD_PER_COUNT / dt
        a = OMEGA_FILTER
        self.omega_l = a * wl + (1 - a) * self.omega_l
        self.omega_r = a * wr + (1 - a) * self.omega_r

        # 오도메트리 (차동 구동)
        dl, dr = dcl * M_PER_COUNT, dcr * M_PER_COUNT
        d = (dl + dr) / 2.0
        dth = self._gyro_dtheta(dt, dl, dr)
        self.x += d * math.cos(self.theta + dth / 2.0)
        self.y += d * math.sin(self.theta + dth / 2.0)
        self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))
        self.distance += abs(d)

        # 속도 제어: 목표를 한 번에 바꾸지 않고 ACCEL_LIMIT 기울기로 서서히 따라감
        step = ACCEL_LIMIT * dt
        for i, goal in enumerate(self.target):
            diff = goal - self.cmd[i]
            self.cmd[i] += max(-step, min(step, diff))
        tl, tr = self.cmd
        if tl == 0.0 and tr == 0.0:
            self.ctl_l.reset()
            self.ctl_r.reset()
            self.pwm_l = self.pwm_r = 0.0
            drive(motor_left, 0.0)
            drive(motor_right, 0.0)
            return cl, cr

        if self.hold_heading and self.imu is not None:
            # 방향 유지: 명령 시작 때의 방향(θ)을 자이로로 붙잡음
            # 왼쪽으로 틀어지면(θ 증가) 오차가 음수 -> 왼쪽 빠르게, 오른쪽 느리게
            err = wrap_angle(self.heading_target - self.theta)
            corr = max(-HEAD_CORR_LIMIT, min(HEAD_CORR_LIMIT, K_HEAD * err))
            tl_adj = tl - corr
            tr_adj = tr + corr
        elif abs(abs(self.target[0]) - abs(self.target[1])) > 1e-6:
            # 곡선 주행: 좌우가 원래 다른 속도이므로 동기화 없이 각 바퀴 속도 제어만
            tl_adj, tr_adj = tl, tr
        else:
            # 동기화: 명령 시작 이후 두 바퀴가 같은 각도만큼 돌도록 보정
            sl, sr = math.copysign(1, self.target[0]), math.copysign(1, self.target[1])
            prog_l = (cl - self._seg_start[0]) * RAD_PER_COUNT * sl
            prog_r = (cr - self._seg_start[1]) * RAD_PER_COUNT * sr
            sync = K_SYNC * (prog_l - prog_r)
            tl_adj = tl - sl * sync
            tr_adj = tr + sr * sync

        self.pwm_l = self.ctl_l.update(tl_adj, self.omega_l, dt)
        self.pwm_r = self.ctl_r.update(tr_adj, self.omega_r, dt)
        drive(motor_left, self.pwm_l)
        drive(motor_right, self.pwm_r)
        return cl, cr

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        if self._gyro_thread is not None:
            self._gyro_thread.join(timeout=1.0)
        motor_left.stop()
        motor_right.stop()
        self.enc_l.close()
        self.enc_r.close()
        if self.imu is not None:
            self.imu.close()
        lgpio.gpiochip_close(self.chip)


# ==========================================
# 7. 화면
# ==========================================
KEYMAP = {
    ord('w'): "FORWARD", ord('W'): "FORWARD",
    ord('s'): "BACKWARD", ord('S'): "BACKWARD",
    ord('a'): "TURN_LEFT", ord('A'): "TURN_LEFT",
    ord('d'): "TURN_RIGHT", ord('D'): "TURN_RIGHT",
    ord(' '): "STOP",
}


def main(stdscr):
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.addstr(0, 0, "자이로 영점 잡는 중... 차를 움직이지 마세요")
    stdscr.refresh()
    robot = Robot()

    try:
        while True:
            key = stdscr.getch()
            if key in (ord('q'), ord('Q')):
                break
            if key in KEYMAP:
                robot.command(KEYMAP[key])

            r = robot
            stdscr.erase()
            stdscr.addstr(0, 0, "================================================")
            stdscr.addstr(1, 0, "     엔코더 오도메트리 + 바퀴 속도 제어         ")
            stdscr.addstr(2, 0, "================================================")
            stdscr.addstr(4, 0, f" [상태] {r.state:<10} | 목표 {TARGET_OMEGA:.2f} rad/s | 자이로 {'ON' if r.imu else 'OFF'} | 모터보정 {'ON' if r.calib else 'OFF'}")
            stdscr.addstr(6, 0, "            왼쪽        오른쪽")
            stdscr.addstr(7, 0, f" 카운트  {r.enc_l.count:>9d}  {r.enc_r.count:>9d}")
            stdscr.addstr(8, 0, f" ω(rad/s) {r.omega_l:>8.2f}  {r.omega_r:>9.2f}")
            stdscr.addstr(9, 0, f" PWM      {r.pwm_l:>8.2f}  {r.pwm_r:>9.2f}")
            stdscr.addstr(11, 0, " ---------------- [ 오도메트리 ] ----------------")
            stdscr.addstr(12, 0, f"  x = {r.x * 100:>7.1f} cm   y = {r.y * 100:>7.1f} cm")
            stdscr.addstr(13, 0, f"  θ = {math.degrees(r.theta):>7.1f} deg  누적거리 {r.distance * 100:>7.1f} cm")
            stdscr.addstr(14, 0, f"  자이로 {r.gyro_dps:>7.2f} deg/s | 튐 {r.gyro_spikes}회 | I2C 에러 {r.i2c_errors}회 ({100.0 * r.i2c_errors / max(1, r.gyro_reads):.1f}%)")
            stdscr.addstr(16, 0, f"  {r.last_error[:60]}")
            stdscr.addstr(15, 0, " [조작] W/S: 전후진 | A/D: 좌우회전 | Space: 정지 | Q: 종료")
            stdscr.refresh()
            time.sleep(0.05)
    finally:
        robot.close()


# ==========================================
# 8. 모터 특성 자동 측정 (python3 파일명.py --calibrate)
# ==========================================
def calibrate():
    """PWM을 단계별로 주고 엔코더로 실제 각속도를 재서, 모터마다
    ω = k × (PWM − dead) 직선을 구함. 바퀴를 바닥에서 띄우고 실행."""
    chip = lgpio.gpiochip_open(GPIO_CHIP)
    enc = {"L": Encoder(chip, ENC_L_A, ENC_L_B, ENC_L_INVERT),
           "R": Encoder(chip, ENC_R_A, ENC_R_B, ENC_R_INVERT)}
    data = {w: {"fwd": [], "bwd": []} for w in "LR"}
    try:
        for d, sign in (("fwd", 1), ("bwd", -1)):
            for u in CALIB_PWMS:
                drive(motor_left, sign * u)
                drive(motor_right, sign * u)
                time.sleep(0.7)                         # 속도가 안정될 때까지 대기
                c0 = {w: e.count for w, e in enc.items()}
                t0 = time.monotonic()
                time.sleep(0.8)
                t1 = time.monotonic()
                line = f"  {d} PWM {u:.2f}:"
                for w, e in enc.items():
                    omega = (e.count - c0[w]) * RAD_PER_COUNT / (t1 - t0) * sign
                    data[w][d].append((u, omega))
                    line += f"  {w} {omega:6.2f} rad/s"
                print(line)
            motor_left.stop()
            motor_right.stop()
            time.sleep(1.0)
    finally:
        motor_left.stop()
        motor_right.stop()
        for e in enc.values():
            e.close()
        lgpio.gpiochip_close(chip)

    result = {}
    for w in "LR":
        result[w] = {}
        for d in ("fwd", "bwd"):
            pts = data[w][d]
            moving = [i for i, (_, om) in enumerate(pts) if om > 0.3]
            if len(moving) < 2:
                neg = any(om < -0.3 for _, om in pts)
                hint = "엔코더 방향(ENC_*_INVERT)을 확인하세요" if neg else "모터나 엔코더 배선을 확인하세요"
                print(f"[실패] {w} {d}: 회전이 거의 측정되지 않음 -> {hint}")
                return
            first = moving[0]
            # 돌기 시작한 직전 PWM을 속도 0인 점으로 (데드존 경계)
            start_u = pts[first - 1][0] if first > 0 else max(0.0, pts[first][0] - 0.02)
            start_u = round(start_u, 3)
            table = [[start_u, 0.0]]
            for u, om in pts[first:]:
                if om > table[-1][1] + 0.05:      # 속도가 오히려 줄거나 안 느는 점은 제외 (포화, 측정 튐)
                    table.append([u, round(om, 3)])
            result[w][d] = table
            print(f"{w} {d}: 돌기 시작 PWM ≈ {start_u:.2f}, 사용 점 {len(table)}개, 최대 {table[-1][1]:.2f} rad/s")

    with open(CALIB_FILE, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(f"저장 완료: {CALIB_FILE}")


if __name__ == '__main__':
    if "--calibrate" in sys.argv:
        calibrate()
    else:
        curses.wrapper(main)
