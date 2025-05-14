import socket
import json
import RPi.GPIO as GPIO
import smbus2
import time
import threading

#----Настройки-----------------------------------
LISTEN_PORT = 5555
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))
#----------------------------------------------
PWM_PIN_LEFT = 26  # Пин для ШИМ левого поворота
PWM_PIN_RIGHT = 20  # Пин для ШИМ правого поворота

PIN_FRONT_LEFT_DRIVE = 19
PIN_FRONT_LEFT_REVERSE = 16
PWM_PIN_FRONT_LEFT =21

ENCODER_ADDRESS = 0x36  # Адрес энкодера AS6500 по I2C
ANGLE_REG = 0x0E  # Адрес чтения значения угла
I2C_BUS = 1  # Номер шины I2C
#-------------------------------------------
old_min = 100
old_max = -100
new_min = 950
new_max = 1840
#---------------------------------------------
state_lock = threading.Lock()
global_state = {
    "steer": 0,
    "throttle": 0,
    "brake": 0,
    "pressed_buttons": []
}

selector = 1

def convert_range(value, old_min, old_max, new_min, new_max):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min

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

        # Инициализация I2C
        self.bus = smbus2.SMBus(I2C_BUS)

        # Инициализация ШИМ
        self.pwm_left = GPIO.PWM(PWM_PIN_LEFT, 100)  
        self.pwm_right = GPIO.PWM(PWM_PIN_RIGHT, 100) 
        self.pwm_front_left = GPIO.PWM(PWM_PIN_FRONT_LEFT, 50)
         
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        
        self.pwm_front_left.start(0)

    def set_speed(self, left_speed, right_speed):
        # left_speed и right_speed должны быть в диапазоне 0-100%
        if left_speed < 0 or left_speed > 100:
            raise ValueError("Скорость левого двигателя должна быть в диапазоне 0-100%")
        if right_speed < 0 or right_speed > 100:
            raise ValueError("Скорость правого двигателя должна быть в диапазоне 0-100%")

        self.pwm_left.ChangeDutyCycle(left_speed)
        self.pwm_right.ChangeDutyCycle(right_speed)

    def read_encoder(self):
        try:
            data = self.bus.read_i2c_block_data(ENCODER_ADDRESS, ANGLE_REG, 2)
            position = (data[0] << 8) | data[1]
            return position
        except Exception as e:
            print(f"Ошибка чтения энкодера: {e}")
            return None

    def turn_to_angle(self, target_angle, current_angle, tolerance):
        if target_angle > 4096 or target_angle < 0 or current_angle is None:
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
          self.pwm_front_left.ChangeDutyCycle(speed)
          GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.HIGH)
          GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.LOW)
          
        if selector == 0:
          self.pwm_front_left.ChangeDutyCycle(speed)
          GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.LOW)
          GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.HIGH)
          
          
        
    def brake(self, speed):
        if speed >0:
          self.pwm_front_left.ChangeDutyCycle(speed)
          GPIO.output(PIN_FRONT_LEFT_DRIVE, GPIO.LOW)
          GPIO.output(PIN_FRONT_LEFT_REVERSE, GPIO.LOW)

    def cleanup(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()

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
                global_state["pressed_buttons"] = [i for i, b in enumerate(state['buttons']) if b]
        except Exception as e:
            print(f"Ошибка обработки пакета: {e}")

receiver_thread = threading.Thread(target=packet_receiver, daemon=True)
receiver_thread.start()

print("[Receiver] G923 receiver started, waiting for data...")

run = True
motor = MotorController()
current_angle = motor.read_encoder()
try:
    run = True

    if current_angle is None:
        current_angle = 0
    step = 40  # Максимальный шаг за один цикл (подберите под  механику)

    while run:
        with state_lock:
            steer = global_state["steer"]
            throttle = global_state["throttle"]
            brake = global_state["brake"]
            pressed_buttons = global_state["pressed_buttons"]

        # Определяем угол поворота колес
        target_angle = round(convert_range(steer, old_min, old_max, new_min, new_max))

        # Вычисляем разницу и ограничиваем шаг
        diff = target_angle - current_angle
        if abs(diff) > step:
            move_angle = current_angle + step if diff > 0 else current_angle - step
        else:
            move_angle = target_angle

        # Применяем движение
        motor.turn_to_angle(move_angle, current_angle, 20)

        # Даем небольшую задержку для завершения движения
        time.sleep(0.01)

        # Читаем актуальный угол после движения
        actual_angle = motor.read_encoder()
        if actual_angle is not None:
            current_angle = actual_angle
        
        if pressed_buttons == [20]:
            selector = 0
        if pressed_buttons == [19]:
            selector = 1    
        motor.drive(throttle, selector)
        motor.brake(brake)

        print(f"[G923] Steering: {steer}° | Throttle: {throttle}% | Brake: {brake}% | Buttons: {pressed_buttons} | Target: {target_angle} | actual: {actual_angle} ")

except KeyboardInterrupt:
    pass
finally:
    motor.cleanup()