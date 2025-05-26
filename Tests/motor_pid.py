import time
import smbus2
import RPi.GPIO as GPIO
from simple_pid import PID

# Настройки AS5600
AS5600_ADDR = 0x36  # I2C адрес AS5600
ANGLE_REG = 0x0C    # Регистр угла (RAW ANGLE)

# Настройки BTS7960
RPWM_PIN = 26       # Пин для правого PWM (GPIO18)
LPWM_PIN = 20       # Пин для левого PWM (GPIO19)
R_EN_PIN = 19       # Пин включения правого направления
L_EN_PIN = 16       # Пин включения левого направления
PWM_FREQ = 1000     # Частота PWM (Гц)

# Настройка GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(RPWM_PIN, GPIO.OUT)
GPIO.setup(LPWM_PIN, GPIO.OUT)
GPIO.setup(R_EN_PIN, GPIO.OUT)
GPIO.setup(L_EN_PIN, GPIO.OUT)

# Инициализация PWM
pwm_r = GPIO.PWM(RPWM_PIN, PWM_FREQ)
pwm_l = GPIO.PWM(LPWM_PIN, PWM_FREQ)
pwm_r.start(0)
pwm_l.start(0)

# Инициализация I2C
bus = smbus2.SMBus(1)

# Настройка PID-регулятора
pid = PID(1.0, 0.1, 0.05, setpoint=0)  # Kp, Ki, Kd
pid.output_limits = (-100, 100)       # Ограничение выхода PWM
pid.sample_time = 0.01                # Частота обновления PID (сек)

def read_as5600_angle():
    """Чтение угла с AS5600 в градусах."""
    try:
        # Чтение двух байт из регистра угла
        low_byte = bus.read_byte_data(AS5600_ADDR, ANGLE_REG + 1)
        high_byte = bus.read_byte_data(AS5600_ADDR, ANGLE_REG)
        raw_angle = (high_byte << 8) | low_byte
        # Преобразование в градусы (12 бит = 4096 уровней)
        angle = (raw_angle * 360.0) / 4096.0
        return angle
    except Exception as e:
        print(f"Ошибка чтения AS5600: {e}")
        return 0.0

def set_motor_pwm(speed, direction):
    """Управление мотором через BTS7960."""
    # Остановка мотора, если скорость близка к нулю
    if abs(speed) < 5:
        pwm_r.ChangeDutyCycle(0)
        pwm_l.ChangeDutyCycle(0)
        GPIO.output(R_EN_PIN, GPIO.LOW)
        GPIO.output(L_EN_PIN, GPIO.LOW)
        return

    # Включение драйвера
    GPIO.output(R_EN_PIN, GPIO.HIGH)
    GPIO.output(L_EN_PIN, GPIO.HIGH)

    # Установка направления и скорости
    if direction > 0:
        pwm_r.ChangeDutyCycle(abs(speed))
        pwm_l.ChangeDutyCycle(0)
    else:
        pwm_r.ChangeDutyCycle(0)
        pwm_l.ChangeDutyCycle(abs(speed))

def main():
    try:
        # Заданный угол (например, 90 градусов)
        target_angle = 90.0
        pid.setpoint = target_angle

        while True:
            # Чтение текущего угла
            current_angle = read_as5600_angle()
            print(f"Текущий угол: {current_angle:.2f}°, Целевой угол: {target_angle:.2f}°")

            # Вычисление управляющего сигнала с помощью PID
            control = pid(current_angle)

            # Управление мотором
            set_motor_pwm(control, control)

            # Задержка для соответствия частоте обновления PID
            time.sleep(pid.sample_time)

    except KeyboardInterrupt:
        print("Программа остановлена")
    finally:
        # Остановка мотора и очистка GPIO
        pwm_r.stop()
        pwm_l.stop()
        GPIO.output(R_EN_PIN, GPIO.LOW)
        GPIO.output(L_EN_PIN, GPIO.LOW)
        GPIO.cleanup()
        bus.close()

if __name__ == "__main__":
    main()