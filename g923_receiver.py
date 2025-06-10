import socket
import json
import RPi.GPIO as GPIO
import time
import threading
import math
from decouple import config

# Настройки
LISTEN_PORT = config('LISTEN_PORT', cast=int)
TIMEOUT_SECONDS = 2.0  # Время ожидания новых данных 
  

# Пины для BTS7960
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

# Пины для JMC integrated Stepper Motor
PUL_PIN = 20  # Пин для сигнала PUL (импульс)
DIR_PIN = 26  # Пин для сигнала DIR (направление)
ENA_PIN = 13  # Пин сигнала Enable(Reset)

# Параметры step двигателя 
STEPS_PER_REVOLUTION = 4000  # Количество шагов на полный оборот (зависит от двигателя)
DELAY = 0.000001  # Задержка между импульсами (в секундах), влияет на скорость
DELAY_INIT = 0.001  # Задержка для инициализации

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
current_steps = -500 # Глобальная переменная количестава начальных шагов

# Инициализация UDP-сокета
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))
sock.settimeout(2.0)

# def convert_range(value, old_min, old_max, new_min, new_max):
#     return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min

class MotorController:
    def __init__(self):
        # Настройка GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
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
        GPIO.setup(PUL_PIN, GPIO.OUT)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(ENA_PIN, GPIO.OUT)
        
        # Сброс возможных ошибок на шаговом моторе
        # GPIO.output(ENA_PIN, GPIO.HIGH)
        # time.sleep(0.2)
        # GPIO.output(ENA_PIN, GPIO.LOW)

        # Инициализация ШИМ
        self.pwm_front_left = GPIO.PWM(PWM_PIN_FRONT_LEFT, 1500)
        self.pwm_front_right = GPIO.PWM(PWM_PIN_FRONT_RIGHT, 1500)
        self.pwm_rear_left = GPIO.PWM(PWM_PIN_REAR_LEFT, 1500)
        self.pwm_rear_right = GPIO.PWM(PWM_PIN_REAR_RIGHT, 1500)
        self.pwm_front_left.start(0)
        self.pwm_front_right.start(0)
        self.pwm_rear_left.start(0)
        self.pwm_rear_right.start(0)

    def rotate_to_position(self, steer):
        global current_steps
        # Преобразование сигнала в угол (45°)
        target_angle = (steer / 100.0) * 45.0
        # Вычисление целевого количества шагов
        target_steps = int((target_angle / 360.0) * STEPS_PER_REVOLUTION)
        # Вычисление необходимого перемещения
        steps_to_move = target_steps - current_steps
        if steps_to_move == 0:
            return  # Нет необходимости двигаться
        # Определение направления
        direction = GPIO.HIGH if steps_to_move > 0 else GPIO.LOW
        GPIO.output(DIR_PIN, direction)
        GPIO.output(ENA_PIN, GPIO.LOW)
        
        # Выполнение шагов
        for _ in range(abs(steps_to_move)):
            GPIO.output(PUL_PIN, GPIO.HIGH)
            time.sleep(DELAY)
            GPIO.output(PUL_PIN, GPIO.LOW)
            time.sleep(DELAY)
        
        # Обновление текущей позиции
        current_steps = target_steps     
    
    def drive(self, speed, selector):
        if speed <= 1: speed =0
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
        if speed >= 1:
            #speed = self.logarithmic(speed)
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

    def tank_turn(self, direction, speed, brake):
        if direction == 0:
            left = 0
            right = 1
        elif direction == 1:
            left = 1
            right = 0
        if speed <= 1: speed =0
        speed = self.logarithmic(speed)
        self.pwm_front_left.ChangeDutyCycle(speed)
        GPIO.output(PIN_FRONT_LEFT_DRIVE, left)
        GPIO.output(PIN_FRONT_LEFT_REVERSE, right)

        self.pwm_front_right.ChangeDutyCycle(speed)
        GPIO.output(PIN_FRONT_RIGHT_DRIVE, right)
        GPIO.output(PIN_FRONT_RIGHT_REVERSE, left)

        self.pwm_rear_left.ChangeDutyCycle(speed)
        GPIO.output(PIN_REAR_LEFT_DRIVE, left)
        GPIO.output(PIN_REAR_LEFT_REVERSE, right)

        self.pwm_rear_right.ChangeDutyCycle(speed)
        GPIO.output(PIN_REAR_RIGHT_DRIVE, right)
        GPIO.output(PIN_REAR_RIGHT_REVERSE, left)



    def logarithmic(self, speed):
        if speed <= 0:
            return 0
        #return min(100, 100 - 100 * math.log10(101 - min(speed, 100)) / math.log10(101))
        return 100 - 100 * math.log10(101 - speed) / math.log10(101)

    def cleanup(self):
        print(f"Starting motor cleanup")
        try:
            for pwm in [self.pwm_front_left, self.pwm_front_right,
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
                global_state = {
                    "steer": 0,
                    "throttle": 0,
                    "brake": 0,
                    "buttons": []
                }
            except KeyError as e:
                print(f"\nОтсутствуют данные: {e}")
                global_state = {
                    "steer": 0,
                    "throttle": 0,
                    "brake": 0,
                    "buttons": []
                }
        except socket.timeout:
            print("\nТаймаут сокета, ожидание новых данных")
            global_state = {
                "steer": 0,
                "throttle": 0,
                "brake": 0,
                "buttons": []
            }
        except Exception as e:
            print(f"\nНепредвиденная ошибка: {e}")

def step_motor_init(): # Выворачиваем колеса влево до упора и ошибки положения и делаем ресет мотора
    global current_steps
    # Сброс возможной ошибки на моторе
    GPIO.output(ENA_PIN, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(ENA_PIN, GPIO.LOW)
    print(f"Инициализация рулевого двигателя...")
    for _ in range(abs(2500)):
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(DELAY_INIT)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(DELAY_INIT)
    print(f"Сброс мотора в начальном положении")
    GPIO.output(ENA_PIN, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(ENA_PIN, GPIO.LOW)
    current_steps = -500

def main():
    global selector
    stop_event = threading.Event()
    receiver_thread = threading.Thread(target=packet_receiver, args=(stop_event,), daemon=True)
    receiver_thread.start()
    motor = MotorController()
    index=1
    step_motor_init()
    time.sleep(0.5)  
    
    try:
        while True:
            with state_lock:
                steer = global_state["steer"]
                throttle = global_state["throttle"]
                brake = global_state["brake"]
                pressed_buttons = global_state["buttons"]
    
            # Обработка кнопок
            if pressed_buttons == [20] or pressed_buttons == [5]:
                selector = 0  # Назад
            elif pressed_buttons == [19] or pressed_buttons == [4]:
                selector = 1  # Вперед
            if pressed_buttons ==[0, 1, 2, 3, 11]:
                step_motor_init()
            if pressed_buttons == [11]:
                motor.tank_turn(0, throttle, brake)
            elif pressed_buttons == [10]:
                motor.tank_turn(1, throttle, brake)
            else:
                # Управление движением
                motor.rotate_to_position(steer)
                if brake <= 5:
                    motor.drive(throttle, selector)
                motor.brake(brake)
            
            # Проверка таймаута
            index += 1
            if time.time() - last_packet_time > TIMEOUT_SECONDS:
                steer = 0
                throttle = 0
                brake = 0
                motor.drive(0, selector)
                motor.brake(0)
                if index % 1500 == 0:
                    print(f"\nОбрыв связи", "Данные не получены, моторы остановлены")
            else:
                if index % 1500 == 0:
                    print(f"{selector=:1d} | {steer=:4d} | {throttle=:3d}% | {brake=:3d}% | {current_steps=:4d} | {pressed_buttons=}")    
            time.sleep(DELAY)
            
    except KeyboardInterrupt:
        stop_event.set()
        print("\nПрограмма остановлена")
    finally:
        motor.cleanup()
        stop_event.set()  # Остановка отдельного потока перед закрытием сокета
        time.sleep(0.5)  # Даем время потоку завершиться
        try:
            sock.close()
        except Exception as e:
            print(f"Ошибка при закрытии сокета: {e}")

if __name__ == "__main__":
    main()