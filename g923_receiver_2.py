import socket
import json
import RPi.GPIO as GPIO
import smbus2
import time
import threading
import math
from decouple import config, Csv

#----Настройки-----------------------------------
LISTEN_PORT = config('LISTEN_PORT', cast=int)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))
sock.settimeout(1.0)
#----------------------------------------------
PWM_PIN_LEFT = 26  # Пин для ШИМ левого поворота
PWM_PIN_RIGHT = 20  # Пин для ШИМ правого поворота
PIN_STEERING_LEFT = 26
PIN_STEERING_RIGHT = 20
PWM_STEERING = 13
#-----------Передний левый мотор-----------------------------------
PIN_FRONT_LEFT_DRIVE = 19
PIN_FRONT_LEFT_REVERSE = 16
PWM_PIN_FRONT_LEFT = 21
#------------Передний правый мотор----------------------------------
PIN_FRONT_RIGHT_DRIVE = 6
PIN_FRONT_RIGHT_REVERSE = 12
PWM_PIN_FRONT_RIGHT = 5
#------------Задний левый мотор----------------------------------
PIN_REAR_LEFT_DRIVE = 22
PIN_REAR_LEFT_REVERSE = 23
PWM_PIN_REAR_LEFT = 24
#------------Задний правый мотор----------------------------------
PIN_REAR_RIGHT_DRIVE = 17
PIN_REAR_RIGHT_REVERSE = 18
PWM_PIN_REAR_RIGHT = 27
#----------------------------------------------
ENCODER_ADDRESS = 0x36  # Адрес энкодера AS6500 по I2C
ANGLE_REG = 0x0E  # Адрес чтения значения угла
I2C_BUS = 1  # Номер шины I2C
#-------------------------------------------
OLD_MIN = 100
OLD_MAX = -100
# NEW_MIN = 950
# NEW_MAX = 1840
NEW_MIN = 1050
NEW_MAX = 1780
#---------------------------------------------
state_lock = threading.Lock()
global_state = {
    "steer": 0,
    "throttle": 0,
    "brake": 0,
    "buttons": []
}

selector = 1

def convert_range(value, OLD_MIN, OLD_MAX, NEW_MIN, NEW_MAX):
    return ((value - OLD_MIN) / (OLD_MAX - OLD_MIN)) * (NEW_MAX - NEW_MIN) + NEW_MIN

class MotorController:
    def __init__(self):
        # Настройка GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN_LEFT, GPIO.OUT)
        GPIO.setup(PWM_PIN_RIGHT, GPIO.OUT)
        
        GPIO.setup(PWM_PIN_FRONT_LEFT, GPIO.OUT)
        GPIO.setup(PIN_FRONT_LEFT_DRIVE, GPIO.OUT)
        GPIO.setup(PIN_FRONT_LEFT_REVERSE, GPIO.OUT)

        GPIO.setup(PWM_PIN_FRONT_RIGHT, GPIO.OUT)
        GPIO.setup(PIN_FRONT_RIGHT_DRIVE, GPIO.OUT)
        GPIO.setup(PIN_FRONT_RIGHT_REVERSE, GPIO.OUT)
        GPIO.setup(PWM_STEERING, GPIO.OUT)
        GPIO.setup(PIN_STEERING_LEFT, GPIO.OUT)
        GPIO.setup(PIN_STEERING_RIGHT, GPIO.OUT)

        # Инициализация I2C
        self.bus = smbus2.SMBus(I2C_BUS)

        # Инициализация ШИМ
        self.pwm_steering = GPIO.PWM(PWM_STEERING, 300)
        
        self.pwm_front_left = GPIO.PWM(PWM_PIN_FRONT_LEFT, 50)
        self.pwm_front_right = GPIO.PWM(PWM_PIN_FRONT_RIGHT, 50)

        self.pwm_steering.start(0)
        
        self.pwm_front_left.start(0)
        self.pwm_front_right.start(0)

    def set_speed(self, left_speed, right_speed):
        # left_speed и right_speed должны быть в диапазоне 0-100%
        if left_speed < 0 or left_speed > 100:
            raise ValueError("Скорость левого двигателя должна быть в диапазоне 0-100%")
        if right_speed < 0 or right_speed > 100:
            raise ValueError("Скорость правого двигателя должна быть в  диапазоне 0-100%")

        self.pwm_steering.ChangeDutyCycle(100)
        GPIO.output(PIN_STEERING_LEFT, left_speed)
        GPIO.output(PIN_STEERING_RIGHT, right_speed)

    def read_encoder(self):
        try:
            data = self.bus.read_i2c_block_data(ENCODER_ADDRESS, ANGLE_REG, 2)
            position = (data[0] << 8) | data[1]
            return position
        except Exception as e:
            print(f"Ошибка чтения энкодера: {e}")
            return None

    def turn_to_angle(self, target_angle, current_angle, tolerance):
        if target_angle >= 4096 or target_angle <= 0 or current_angle is None:
            self.set_speed(0, 0)
            return

        diff = target_angle - current_angle
        if abs(diff) <= tolerance:
            self.set_speed(0, 0)  # Остановка
        elif diff > 0:
            self.set_speed(98, 0)  # Поворот влево
        else:
            self.set_speed(0, 98)  # Поворот вправо
            
            
    def drive(self, speed, selector):
        if selector == 1:
          self.pwm_front_left.ChangeDutyCycle(self.logarifmic(speed))
          GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.HIGH)
          GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.LOW)

          self.pwm_front_right.ChangeDutyCycle(self.logarifmic(speed))
          GPIO.output(PIN_FRONT_RIGHT_DRIVE, GPIO.HIGH)
          GPIO.output(PIN_FRONT_RIGHT_REVERSE, GPIO.LOW)
          
        if selector == 0:
          self.pwm_front_left.ChangeDutyCycle(self.logarifmic(speed))
          GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.LOW)
          GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.HIGH)

          self.pwm_front_right.ChangeDutyCycle(self.logarifmic(speed))
          GPIO.output(PIN_FRONT_RIGHT_DRIVE, GPIO.LOW)
          GPIO.output(PIN_FRONT_RIGHT_REVERSE, GPIO.HIGH)
          
    def logarifmic(self, speed):
        return 100 - 100 * math.log10(101 - speed) / math.log10(101)
        #log_value = math.log10(101-speed)
        #normalized_log_value = (log_value - math.log10(101)) / (math.log10(101) - math.log10(1))
        #return ((1 - normalized_log_value) * 100) -100

    def brake(self, speed):
        if speed > 0:
          self.pwm_front_left.ChangeDutyCycle(self.logarifmic(speed))
          GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.LOW)
          GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.LOW)

          self.pwm_front_right.ChangeDutyCycle(self.logarifmic(speed))
          GPIO.output(PIN_FRONT_RIGHT_DRIVE, GPIO.LOW)
          GPIO.output(PIN_FRONT_RIGHT_REVERSE, GPIO.LOW)

    def cleanup(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        sock.close()
"""#------------------------------------
def packet_receiver():
    global global_state
    while True:
        data, _ = sock.recvfrom(2048)
        try:
            state = json.loads(data)
            with state_lock:
                global_state["steer"] = state['axes']['steer']
                global_state["throttle"] = state['axes']['throttle']
                global_state["brake"] = state['axes']['brake']
                global_state["buttons"] = state['buttons']

        except Exception as e:
            print(f"Ошибка обработки пакета: {e}")

receiver_thread = threading.Thread(target=packet_receiver, daemon=True)
receiver_thread.start()
#------------------------------------"""
def packet_receiver(stop_event):
    global global_state
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(2048)
            try:
                state = json.loads(data.decode('utf-8'))
                with state_lock:
                    global_state["steer"] = state['axes'].get('steer', global_state.get('steer', 0))
                    global_state["throttle"] = state['axes'].get('throttle', global_state.get('throttle', 0))
                    global_state["brake"] = state['axes'].get('brake', global_state.get('brake', 0))
                    global_state["buttons"] = state.get('buttons', global_state.get('buttons', {}))
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except KeyError as e:
                print(f"Missing key in JSON data: {e}")
        except socket.error as e:
            if not stop_event.is_set():
                print(f"Socket error: {e}")
                global_state = {
                    "steer": 0,
                    "throttle": 0,
                    "brake": 0,
                    "buttons": []
                }
        except Exception as e:
            print(f"Unexpected error: {e}")
            
stop_event = threading.Event()
receiver_thread = threading.Thread(target=packet_receiver, args=(stop_event,), daemon=True)
receiver_thread.start()


print("[Receiver] G923 receiver started, waiting for data...")

run = True
motor = MotorController()
current_angle = motor.read_encoder()
try:
    run = True

    if current_angle is None:
        current_angle = 0
    step = 50  # Максимальный шаг за один цикл (подберите под  механику)

    while run:
        with state_lock:
            steer = global_state["steer"]
            throttle = global_state["throttle"]
            brake = global_state["brake"]
            pressed_buttons = global_state["buttons"]

        # Определяем угол поворота колес
        target_angle = round(convert_range(steer, OLD_MIN, OLD_MAX, NEW_MIN, NEW_MAX))

        # Вычисляем разницу и ограничиваем шаг
        diff = target_angle - current_angle
        if abs(diff) > step:
            move_angle = current_angle + step if diff > 0 else current_angle - step
        else:
            move_angle = target_angle

        # Применяем движение
        motor.turn_to_angle(move_angle, current_angle, 45)

        # Даем небольшую задержку для завершения движения
        time.sleep(0.01)

        # Читаем актуальный угол после движения
        actual_angle = motor.read_encoder()
        if actual_angle is not None:
            current_angle = actual_angle
        
        if pressed_buttons == [20] or pressed_buttons == [5]:
            selector = 0
        if pressed_buttons == [19] or pressed_buttons == [4]:
            selector = 1    
        motor.drive(throttle, selector)
        motor.brake(brake)

        print(f"[G923] Steering: {steer}° | Throttle: {throttle}% | Brake: {brake}% | Buttons: {pressed_buttons} | Target: {target_angle} | actual: {actual_angle} ")

except KeyboardInterrupt:
    pass
finally:
    motor.cleanup()
