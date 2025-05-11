import RPi.GPIO as GPIO
import time
import smbus
from pid import PIDController

# Настройки пинов
PWM_PIN_LEFT = 13  # Пин для ШИМ левого поворота
PWM_PIN_RIGHT = 23  # Пин для ШИМ правого поворота
ENCODER_ADDRESS = 0x40  # Адрес энкодера AS6500 по I2C
I2C_BUS = 1  # Номер шины I2C

class MotorController:
    def __init__(self):
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN_LEFT, GPIO.OUT)
        GPIO.setup(PWM_PIN_RIGHT, GPIO.OUT)

        # Инициализация I2C
        self.bus = smbus.SMBus(I2C_BUS)

        # Инициализация ШИМ
        self.pwm_left = GPIO.PWM(PWM_PIN_LEFT, 100)  # Частота 100 Гц
        self.pwm_right = GPIO.PWM(PWM_PIN_RIGHT, 100)  # Частота 100 Гц
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Настройка PID-регулятора
        self.pid = PIDController(1, 0.1, 0.05, setpoint=0)

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
            data = self.bus.read_i2c_block_data(ENCODER_ADDRESS, 0, 2)
            position = (data[0] << 8) | data[1]
            return position
        except Exception as e:
            print(f"Ошибка чтения энкодера: {e}")
            return None

    def turn_to_angle(self, target_angle):
        current_angle = self.read_encoder()
        if current_angle is None:
            return

        while abs(target_angle - current_angle) > 1:
            error = target_angle - current_angle
            control_signal = self.pid.update(error)

            if control_signal > 0:
                self.set_speed(abs(control_signal), 0)  # Поворот влево
            else:
                self.set_speed(0, abs(control_signal))  # Поворот вправо

            time.sleep(0.01)
            current_angle = self.read_encoder()

        self.set_speed(0, 0)  # Остановка

    def cleanup(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()

# Пример использования
if __name__ == "__main__":
    try:
        motor = MotorController()

        # Поворот на заданный угол
        target_angle = 1000  # Пример угла
        motor.turn_to_angle(target_angle)

        # Чтение текущей позиции энкодера
        print(f"Текущая позиция: {motor.read_encoder()}")

    except KeyboardInterrupt:
        pass
    finally:
        motor.cleanup()
