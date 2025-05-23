import RPi.GPIO as GPIO
import time

# Настройка GPIO пинов
PUL_PIN = 25  # Пин для сигнала PUL (импульс)
DIR_PIN = 23  # Пин для сигнала DIR (направление)

# Параметры двигателя
STEPS_PER_REVOLUTION = 500  # Количество шагов на полный оборот (зависит от двигателя)
MICROSTEPS = 8  # Количество микрошагов, настроенных на драйвере
DELAY = 0.00001  # Задержка между импульсами (в секундах), влияет на скорость

# Вычисление количества шагов на градус
STEPS_PER_DEGREE = (STEPS_PER_REVOLUTION * MICROSTEPS) / 360.0

# Инициализация GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

def rotate_motor(angle, direction):
    """Функция для поворота двигателя на заданный угол.
    angle: угол поворота в градусах
    direction: True для вращения по часовой стрелке, False - против"""
    
    # Установка направления
    GPIO.output(DIR_PIN, direction)
    
    # Вычисление количества импульсов
    steps = int(abs(angle) * STEPS_PER_DEGREE)
    
    # Генерация импульсов
    for _ in range(steps):
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(DELAY)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(DELAY)

def main():
    try:
        while True:
            # Запрос угла поворота
            angle = float(input("Введите угол поворота в градусах (положительный или отрицательный): "))
            
            # Определение направления
            direction = True if angle >= 0 else False
            
            print(f"Поворот на {abs(angle)} градусов {'по часовой стрелке' if direction else 'против часовой стрелки'}")
            rotate_motor(angle, direction)
            
    except KeyboardInterrupt:
        print("\nПрограмма завершена")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()