import socket
import json
import RPi.GPIO as GPIO
import smbus2
import time
import threading
import math
from decouple import config

# Настройки
LISTEN_PORT = config('LISTEN_PORT', cast=int)
TIMEOUT_SECONDS = 2.0  # Время ожидания новых данных

# Пины для BTS7960
PIN_STEERING_LEFT = 26
PIN_STEERING_RIGHT = 20
PWM_STEERING = 13
PIN_FRONT_LEFT_DRIVE = 19
PIN_FRONT_LEFT_REVERSE = 16
PWM_PIN_FRONT_LEFT = 21
PIN_FRONT_RIGHT_DRIVE = 6
PIN_FRONT_RIGHT_REVERSE = 12
PWM_PIN_FRONT_RIGHT = 5
PIN_REAR_LEFT_DRIVE = 22
PIN_REAR_LEFT_REVERSE = 23
PWM_PIN_REAR_LEFT = 24
PIN_REAR_RIGHT_DRIVE = 17
PIN_REAR_RIGHT_REVERSE = 18
PWM_PIN_REAR_RIGHT = 27

# Конфигурация I2C для AS5600
ENCODER_ADDRESS = 0x36
ANGLE_REG = 0x0C  # RAW ANGLE
I2C_BUS = 1

# Диапазон преобразования угла в значения 12 бит энкодера
OLD_MIN = -100
OLD_MAX = 100
NEW_MIN = 250
NEW_MAX = 2250

# Параметры ПИД
KP = 0.5
KI = 0.9
KD = 0.3
INTEGRAL_LIMIT = 100
MAX_DUTY = 100
ANGLE_TOLERANCE = 80
STEP = 40
DT = 0.001

# Глобальное состояние
global_state = {
    "steer": 0,
    "throttle": 0,
    "brake": 0,
    "buttons": []
}
state_lock = threading.Lock()
last_packet_time = time.time()
selector = 1  # Глобальная переменная для направления движения

# Инициализация UDP-сокета
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))
sock.settimeout(2.0)

def convert_range(value, old_min, old_max, new_min, new_max):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min

class MotorController:
    def __init__(self, kp, ki, kd, dt, integral_limit, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral_limit = integral_limit
        self.max_output = max_output
        self.integral = 0
        self.prev_error = 0

        # Настройка GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_STEERING, GPIO.OUT)
        GPIO.setup(PIN_STEERING_LEFT, GPIO.OUT)
        GPIO.setup(PIN_STEERING_RIGHT, GPIO.OUT)
        GPIO.setup(PWM_PIN_FRONT_LEFT, GPIO.OUT)
        GPIO.setup(PIN_FRONT_LEFT_DRIVE, GPIO.OUT)
        GPIO.setup(PIN_FRONT_LEFT_REVERSE, GPIO.OUT)
        GPIO.setup(PWM_PIN_FRONT_RIGHT, GPIO.OUT)
        GPIO.setup(PIN_FRONT_RIGHT_DRIVE, GPIO.OUT)
        GPIO.setup(PIN_FRONT_RIGHT_REVERSE, GPIO.OUT)
        GPIO.setup(PWM_PIN_REAR_LEFT, GPIO.OUT)
        GPIO.setup(PIN_REAR_LEFT_DRIVE, GPIO.OUT)
        GPIO.setup(PIN_REAR_LEFT_REVERSE, GPIO.OUT)
        GPIO.setup(PWM_PIN_REAR_RIGHT, GPIO.OUT)
        GPIO.setup(PIN_REAR_RIGHT_DRIVE, GPIO.OUT)
        GPIO.setup(PIN_REAR_RIGHT_REVERSE, GPIO.OUT)

        # Инициализация I2C
        self.bus = smbus2.SMBus(I2C_BUS)

        # Инициализация ШИМ
        self.pwm_steering = GPIO.PWM(PWM_STEERING, 50) 
        self.pwm_front_left = GPIO.PWM(PWM_PIN_FRONT_LEFT, 50)
        self.pwm_front_right = GPIO.PWM(PWM_PIN_FRONT_RIGHT, 50)
        self.pwm_rear_left = GPIO.PWM(PWM_PIN_REAR_LEFT, 50)
        self.pwm_rear_right = GPIO.PWM(PWM_PIN_REAR_RIGHT, 50)

        self.pwm_steering.start(0)
        self.pwm_front_left.start(0)
        self.pwm_front_right.start(0)
        self.pwm_rear_left.start(0)
        self.pwm_rear_right.start(0)

    def calc_pwm(self, target, current, dt):
        error = target - current
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        output = p_term + i_term + d_term
        output = abs(max(min(output, self.max_output), -self.max_output))
        return int(output)

    def set_speed(self, left_speed, right_speed, pwm_signal):
        if pwm_signal < 0 or pwm_signal > 100:
            print(f"\n {pwm_signal=} двигателя должна быть в диапазоне 0-100%")
            raise ValueError("Скорость двигателя должна быть в диапазоне 0-100%")
        self.pwm_steering.ChangeDutyCycle(pwm_signal)
        GPIO.output(PIN_STEERING_LEFT, left_speed)
        GPIO.output(PIN_STEERING_RIGHT, right_speed)

    def read_encoder(self):
        try:
            data = self.bus.read_i2c_block_data(ENCODER_ADDRESS, ANGLE_REG, 2)
            position = (data[0] << 8) | data[1]
            return position
        except Exception as e:
            print(f"\nОшибка чтения энкодера: {e}")
            return None

    def turn_to_angle(self, target_angle, tolerance):
        current_angle = self.read_encoder()
        if current_angle is None:
            self.set_speed(0, 0, 0)
            return
        diff = target_angle - current_angle
        pwm_signal = self.calc_pwm(target_angle, current_angle, self.dt)
        # print(f"{current_angle=:4d}", end = " | ")
        # print(f"{diff=:4d}", end=" | ")
        # print(f"{pwm_signal=:4d}", end="\n")
        if target_angle >= 4000 or target_angle <= 100 or abs(diff) <= tolerance:
            self.set_speed(0, 0, 0)
            return pwm_signal, diff, current_angle
        if abs(diff) > STEP:
            target_angle = current_angle + STEP if diff > 0 else current_angle - STEP
            if diff > 0:
                self.set_speed(1, 0, pwm_signal)  # Поворот влево
            else:
                self.set_speed(0, 1, pwm_signal)  # Поворот вправо
        return pwm_signal, diff, current_angle
    def drive(self, speed, selector):
        speed = self.logarithmic(speed)
        if selector == 1:  # Вперед
            self.pwm_front_left.ChangeDutyCycle(speed)
            GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.HIGH)
            GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.LOW)
            self.pwm_front_right.ChangeDutyCycle(speed)
            GPIO.output(PIN_FRONT_RIGHT_DRIVE, GPIO.HIGH)
            GPIO.output(PIN_FRONT_RIGHT_REVERSE, GPIO.LOW)
            self.pwm_rear_left.ChangeDutyCycle(speed)
            GPIO.output(PIN_REAR_LEFT_DRIVE, GPIO.HIGH)
            GPIO.output(PIN_REAR_LEFT_REVERSE, GPIO.LOW)
            self.pwm_rear_right.ChangeDutyCycle(speed)
            GPIO.output(PIN_REAR_RIGHT_DRIVE, GPIO.HIGH)
            GPIO.output(PIN_REAR_RIGHT_REVERSE, GPIO.LOW)
        elif selector == 0:  # Назад
            self.pwm_front_left.ChangeDutyCycle(speed)
            GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.LOW)
            GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.HIGH)
            self.pwm_front_right.ChangeDutyCycle(speed)
            GPIO.output(PIN_FRONT_RIGHT_DRIVE, GPIO.LOW)
            GPIO.output(PIN_FRONT_RIGHT_REVERSE, GPIO.HIGH)
            self.pwm_rear_left.ChangeDutyCycle(speed)
            GPIO.output(PIN_REAR_LEFT_DRIVE, GPIO.LOW)
            GPIO.output(PIN_REAR_LEFT_REVERSE, GPIO.HIGH)
            self.pwm_rear_right.ChangeDutyCycle(speed)
            GPIO.output(PIN_REAR_RIGHT_DRIVE, GPIO.LOW)
            GPIO.output(PIN_REAR_RIGHT_REVERSE, GPIO.HIGH)

    def brake(self, speed):
        speed = self.logarithmic(speed)
        self.pwm_front_left.ChangeDutyCycle(speed)
        GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.LOW)
        GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.LOW)
        self.pwm_front_right.ChangeDutyCycle(speed)
        GPIO.output(PIN_FRONT_RIGHT_DRIVE, GPIO.LOW)
        GPIO.output(PIN_FRONT_RIGHT_REVERSE, GPIO.LOW)
        self.pwm_rear_left.ChangeDutyCycle(speed)
        GPIO.output(PIN_REAR_LEFT_DRIVE, GPIO.LOW)
        GPIO.output(PIN_REAR_LEFT_REVERSE, GPIO.LOW)
        self.pwm_rear_right.ChangeDutyCycle(speed)
        GPIO.output(PIN_REAR_RIGHT_DRIVE, GPIO.LOW)
        GPIO.output(PIN_REAR_RIGHT_REVERSE, GPIO.LOW)

    def logarithmic(self, speed):
        if speed <= 0:
            return 0
        return min(100, 100 - 100 * math.log10(101 - min(speed, 100)) / math.log10(101))

    def cleanup(self):
        print(f"Starting motor cleanup")
        try:
            for pwm in [self.pwm_steering, self.pwm_front_left, self.pwm_front_right,
                       self.pwm_rear_left, self.pwm_rear_right]:
                try:
                    pwm.ChangeDutyCycle(0)
                    pwm.stop()
                except Exception as e:
                    print(f"Error stopping PWM: {e}")
        except Exception as e:
            print(f"Error during PWM cleanup: {e}")
        try:
            GPIO.cleanup()
            print(f"GPIO cleanup completed")
        except Exception as e:
            print(f"Error during GPIO cleanup: {e}")

def packet_receiver(stop_event):
    global global_state, last_packet_time
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(2048)
            try:
                state = json.loads(data.decode('utf-8'))
                with state_lock:
                    global_state["steer"] = state['axes'].get('steer', global_state.get('steer', 0))
                    global_state["throttle"] = state['axes'].get('throttle', global_state.get('throttle', 0))
                    global_state["brake"] = state['axes'].get('brake', global_state.get('brake', 0))
                    global_state["buttons"] = state.get('buttons', global_state.get('buttons', []))
                    last_packet_time = time.time()
            except json.JSONDecodeError as e:
                print(f"\nОшибка JSON: {e}")
            except KeyError as e:
                print(f"\nОтсутствуют данные: {e}")
        except socket.timeout:
            print("\nТаймаут сокета, ожидание новых данных")
        except Exception as e:
            print(f"\nНепредвиденная ошибка: {e}")

def main():
    global selector
    stop_event = threading.Event()
    receiver_thread = threading.Thread(target=packet_receiver, args=(stop_event,), daemon=False)
    receiver_thread.start()
    motor = MotorController(KP, KI, KD, DT, INTEGRAL_LIMIT, MAX_DUTY)
    index=1
    
    
    try:
        while True:
            with state_lock:
                steer = global_state["steer"]
                throttle = global_state["throttle"]
                brake = global_state["brake"]
                pressed_buttons = global_state["buttons"]
            
            # Определяем угол поворота колес
            target_angle = round(convert_range(steer, OLD_MIN, OLD_MAX, NEW_MIN, NEW_MAX))
            
            # Управление движением
            pwm_signal, diff, current_angle=motor.turn_to_angle(target_angle, ANGLE_TOLERANCE)
            motor.drive(throttle, selector)
            motor.brake(brake)
            
            # Обработка кнопок для переключения направления
            if pressed_buttons == [20] or pressed_buttons == [5]:
                selector = 0  # Назад
            elif pressed_buttons == [19] or pressed_buttons == [4]:
                selector = 1  # Вперед
            index += 1
            # Проверка таймаута
            if time.time() - last_packet_time > TIMEOUT_SECONDS:
                steer = 0
                throttle = 0
                brake = 0
                motor.set_speed(0, 0, 0)
                motor.drive(0, selector)
                motor.brake(0)
                if index % 500 == 0:
                    print(f"\nОбрыв связи", "Данные не получены, моторы остановлены")
            else:
                if index % 100 == 0:
                    print(f"{selector=:1d} | {steer=:4d} | {throttle=:3d}% | {brake=:3d}% | {target_angle=:4d} | {current_angle=:4d} | {diff=:4d} | {pwm_signal=:3d} | {pressed_buttons=}")
            
            #time.sleep(DT)
            
    except KeyboardInterrupt:
        stop_event.set()
        print("\nПрограмма остановлена")
    finally:
        motor.cleanup()
        stop_event.set()  # Остановка потока перед закрытием сокета
        time.sleep(0.2)  # Даем время потоку завершиться
        try:
            sock.close()
        except Exception as e:
            print(f"Ошибка при закрытии сокета: {e}")

if __name__ == "__main__":
    main()