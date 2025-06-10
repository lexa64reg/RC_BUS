import sys
sys.path.append('/logidrivepy')
from logidrivepy import LogitechController
import socket
import json
import time
import os
from plyer import notification
from decouple import config, Csv
import subprocess
import keyboard
#-------Настройки---------------
SERVER_IP = config('SERVER_IP')
SERVER_PORT = config('SERVER_PORT', cast=int)
STEERING_MAX_DEGREES = 300  # угол поворота руля 
DAMPER_FORCE = 100  # усилие 0-100%
OLD_MIN = -STEERING_MAX_DEGREES
OLD_MAX = STEERING_MAX_DEGREES
NEW_MIN = -100
NEW_MAX = 100
#------Инициализация----------------
os.environ["SDL_VIDEODRIVER"] = "dummy"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#-------------------------------------------
command = ["python", "video_stream.py"]

def normalize_axis(val):
    return (val / 32767)

def axis_to_percent(val):
    return int((1 - val) * 50) 

def axis_to_degrees(val):
    value = int(val * STEERING_MAX_DEGREES)
    return ((value - OLD_MIN) / (OLD_MAX - OLD_MIN)) * (NEW_MAX - NEW_MIN) + NEW_MIN
#----------------------------------
def show_notification(title, message):
    notification.notify(
        title=title,
        message=message,
        app_name="RC_BUS",
        app_icon="bus.ico",
        timeout=10  # время показа в секундах
    )
#----------------------------------
controller = LogitechController()
if not controller.steering_initialize(False):
    print(f"Руль не найден, выход")
    show_notification("Ошибка!", "Руль не найден")
    exit(1)

if not controller.is_connected(0):
    controller.steering_shutdown()
    print(f"Руль не найден, выход")
    show_notification("Ошибка!", "Руль не найден")
    exit(1)

try:
    print(f"Запуск видеопотока...")
    stream = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=sys.stderr, text=True)
    controller.set_operating_range(0, STEERING_MAX_DEGREES)
    # controller.play_spring_force(0, 0, 100, 40)
    index = 1
    run = True
    while run:
        index += 1
        controller.logi_update()
        state = controller.get_state_engines(0).contents
        if not controller.is_connected(0):
            print(f"Руль не найден, выход")
            steer_deg = 0
            throttle_pct = 0
            brake_pct = 0
            clutch_pct = 0
            controller.steering_shutdown()
            stream.terminate()
            stream.wait()
            run = False
            exit(1)
            show_notification("Ошибка!", "Руль не найден")
            break

        raw_steer = state.lX
        raw_throttle = state.lZ
        raw_clutch = state.lY
        raw_brake = state.lRz

        buttons = [i for i in range(25) if state.rgbButtons[i] != 0]

        steer_deg = int(axis_to_degrees(normalize_axis(raw_steer)))
        throttle_pct = axis_to_percent(normalize_axis(raw_throttle))
        brake_pct = axis_to_percent(normalize_axis(raw_brake))
        clutch_pct = axis_to_percent(normalize_axis(raw_clutch))

        if brake_pct > 80 or any(element in [4, 5, 19, 20] for element in buttons):
            controller.play_bumpy_road_effect(0, 2)
        else:
            controller.stop_bumpy_road_effect(0)
        
        controller.play_leds(0, throttle_pct, 10, 140) 

        controller.play_damper_force(0, int((100 - throttle_pct) / 2)) # Больше газ свободнее руль
        if throttle_pct > 10:
            controller.play_spring_force(0, 0, int(throttle_pct / 5), int(throttle_pct / 10))
        else:
            controller.stop_spring_force(0)

        axes = {
            'steer': steer_deg,
            'throttle': throttle_pct,
            'brake': brake_pct,
            'clutch': clutch_pct
        }
        payload = {
            'axes': axes,
            'buttons': buttons
        }

        # Отправка
        sock.sendto(json.dumps(payload).encode(), (SERVER_IP, SERVER_PORT))
        if index % 500 == 0:
            print(f"Steering: {steer_deg:4d}° | Throttle: {throttle_pct:3d}% | Brake: {brake_pct:3d}% | Clutch: {clutch_pct:3d} | {buttons=}")
                    
        time.sleep(0.01)
        
        if keyboard.is_pressed('r') or keyboard.is_pressed('R'): 
            if stream.poll() is not None:
                print("Перезапуск потока")
                stream = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=sys.stdout, stderr=sys.stderr, text=True)
        elif keyboard.is_pressed('esc'): 
            stream.terminate()
            stream.wait()
            run = False

except KeyboardInterrupt:
    print('exit')
    
finally:
    sock.close()
    controller.steering_shutdown()