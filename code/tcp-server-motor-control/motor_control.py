import RPi.GPIO as GPIO
from time import sleep

## 회전 방식 : 주행 방식은 '직진' or '후진' 
## 좌/우회전의 경우 바퀴의 회전차를 통해 차체를 회전시키는 방식.(근데 누른 시간 만큼 회전할지 아니면 90도 절대값으로 둘지 고민)

# 모터 상태(stat)
STOP, FORWARD, BACKWORD  = 0, 1, 2

# 차체 이동 방향
FRONT, BACK, LEFT, RIGHT, STOP_DRIVE = 0, 1, 2, 3, 4

## 모터 속도(0~100% : 펄스 한 주기에서 신호가 HIGH인 비율)
#SPEED = 10

# 모터 채널
LEFT_CH, RIGHT_CH = 0, 1

# # PWM 주파수(Hz)
# Freq = 100

# PIN 설정
LOW, HIGH = 0, 1

# 실제 핀 정의
#PWM PIN
ENA = 12
ENB = 13

#GPIO PIN
IN1 = 23
IN2 = 24
IN3 = 17
IN4 = 27
pwmA, pwmB = None, None

## 핀 설정 함수
def setPinConfig(EN, INA, INB, freq):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    
    # PWM 객체 생성
    pwm = GPIO.PWM(EN, freq) 
    
    # 우선 PWM 멈춤.   
    pwm.start(0) 
    return pwm



# 모터 제어 함수
def setMotorContorl(pwm, INA, INB, speed, stat):

    # 모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)  
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    # 뒤로
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    # 정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

# 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
def setMotor(ch, speed, stat):
    global pwmA, pwmB
    if ch == LEFT_CH:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)

def setMotor_driving(speed, driving_mode):
        if driving_mode == FRONT:
            setMotor(LEFT_CH, speed, FORWARD)
            setMotor(RIGHT_CH, speed, FORWARD)
            
        elif driving_mode == BACK:
            setMotor(LEFT_CH, speed, BACKWORD)
            setMotor(RIGHT_CH, speed, BACKWORD)
            
        elif driving_mode == LEFT:
            setMotor(LEFT_CH, speed, STOP)
            setMotor(RIGHT_CH, speed, FORWARD)
            
        elif driving_mode == RIGHT:
            setMotor(LEFT_CH, speed, FORWARD)
            setMotor(RIGHT_CH, speed, STOP)

        else:
            setMotor(LEFT_CH, speed, STOP)
            setMotor(RIGHT_CH, speed, STOP)



def init_set_motor(freq):
    global pwmA, pwmB
    
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    #모터 핀 설정
    #핀 설정후 PWM 핸들 얻어옴 
    pwmA = setPinConfig(ENA, IN1, IN2, freq)
    pwmB = setPinConfig(ENB, IN3, IN4, freq)

  