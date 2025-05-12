import socket
import json
import RPi.GPIO as GPIO
import smbus2
import time

#----Настройки-----------------------------------
LISTEN_PORT = 5555
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))
#----------------------------------------------
PWM_PIN_LEFT = 13  # Пин для ШИМ левого поворота
PWM_PIN_RIGHT = 19  # Пин для ШИМ правого поворота
ENCODER_ADDRESS = 0x36  # Адрес энкодера AS6500 по I2C
ANGLE_REG = 0x0E  # Адрес чтения значения угла
I2C_BUS = 1  # Номер шины I2C
#-------------------------------------------
old_min = -100
old_max = 100
new_min = 200
new_max = 3500
#---------------------------------------------


#def read_angle():
#    # Read two bytes from the angle register
#    raw_data = bus.read_i2c_block_data(ENCODER_ADDRESS, ANGLE_REG, 2)
#    angle = (raw_data[0] << 8) | raw_data[1]  # Combine MSB and LSB
#    angle = angle & 0x0FFF  # Mask to 12 bits
#    return (angle / 4096.0) * 360.0  # Convert to degrees
    
    

def convert_range(value, old_min, old_max, new_min, new_max):
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min


class MotorController:
    def __init__(self):
        # Настройка GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN_LEFT, GPIO.OUT)
        GPIO.setup(PWM_PIN_RIGHT, GPIO.OUT)

        # Инициализация I2C
        self.bus = smbus2.SMBus(I2C_BUS)

        # Инициализация ШИМ
        self.pwm_left = GPIO.PWM(PWM_PIN_LEFT, 300)  
        self.pwm_right = GPIO.PWM(PWM_PIN_RIGHT, 300)  
        self.pwm_left.start(0)
        self.pwm_right.start(0)

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

    def turn_to_angle(self, target_angle):
        current_angle = self.read_encoder()
        if current_angle is None:
            return

        while abs(target_angle - current_angle) > 100:
            #print(f"Текущая позиция: {motor.read_encoder()}")
            if target_angle > 4096 or target_angle < 0:
                return
            if target_angle > current_angle:
                self.set_speed(100, 0)  # Поворот влево
            else:
                self.set_speed(0, 100)  # Поворот вправо

            #time.sleep(0.01)
            current_angle = self.read_encoder()

        self.set_speed(0, 0)  # Остановка

    def cleanup(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()


print("[Receiver] G923 receiver started, waiting for data...")


run = True
motor = MotorController()
try:
    #motor = MotorController()
    run = True
    while run:

        
        data, _ = sock.recvfrom(2048)
        state = json.loads(data)

        steer = state['axes']['steer']
        throttle = state['axes']['throttle']
        brake = state['axes']['brake']
        pressed_buttons = [i for i, b in enumerate(state['buttons']) if b]
           
        
        # Поворот на заданный угол
        target_angle = round(convert_range(steer, old_min, old_max, new_min, new_max))
        motor.turn_to_angle(target_angle)
        print(f"[G923] Steering: {steer}° | Throttle: {throttle}% | Brake: {brake}% | Buttons: {pressed_buttons} | Angle: {motor.read_encoder()} degrees | target: {target_angle}")
        print(f"Текущая позиция: {motor.read_encoder()}")

except KeyboardInterrupt:
    pass
finally:
    motor.cleanup()
    