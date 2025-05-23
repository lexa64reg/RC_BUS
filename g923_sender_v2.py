import sys
sys.path.append('/logidrivepy')
from logidrivepy import LogitechController
import socket
import json
import time
import os
from decouple import config, Csv

#-------Настройки---------------
SERVER_IP = config('SERVER_IP')
SERVER_PORT = config('SERVER_PORT', cast=int)
STEERING_MAX_DEGREES = 180 # угол поворота руля
DAMPER_FORCE = 100  # усилие 0-100%
OLD_MIN = -STEERING_MAX_DEGREES
OLD_MAX = STEERING_MAX_DEGREES
NEW_MIN = -100
NEW_MAX = 100
#------Инициализация----------------
os.environ["SDL_VIDEODRIVER"] = "dummy"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#-------------------------------------------
def normalize_axis(val):
    return (val / 32767)

def axis_to_percent(val):
    return int((1 - val) * 50) 

def axis_to_degrees(val):
    value = int(val * STEERING_MAX_DEGREES)
    return ((value - OLD_MIN) / (OLD_MAX - OLD_MIN)) * (NEW_MAX - NEW_MIN) + NEW_MIN
#----------------------------------

controller = LogitechController()
if not controller.steering_initialize(False):
    print(f"Руль не найден, выход")
    exit(1)

if not controller.is_connected(0):
    controller.steering_shutdown()
    print(f"Руль не найден, выход")
    exit(1)

try:
    controller.set_operating_range(0, STEERING_MAX_DEGREES)
    # controller.play_spring_force(0, 0, 100, 40)
    run = True
    while run:
        controller.logi_update()
        state = controller.get_state_engines(0).contents

        raw_steer = state.lX
        raw_throttle = state.lZ
        raw_clutch = state.lY
        raw_brake = state.lRz

        buttons = [i for i in range(25) if state.rgbButtons[i] != 0]

        steer_deg = int(axis_to_degrees(normalize_axis(raw_steer)))
        throttle_pct = axis_to_percent(normalize_axis(raw_throttle))
        brake_pct = axis_to_percent(normalize_axis(raw_brake))
        clutch_pct = axis_to_percent(normalize_axis(raw_clutch))


        if brake_pct > 80:
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

        print(f"[Steering: {steer_deg}° | Throttle: {throttle_pct:3d}% | Brake: {brake_pct:3d}%] | Clutch: {clutch_pct} | {buttons} ")
        time.sleep(0.01)
        

except KeyboardInterrupt:
    print('exit')
    
finally:
    controller.steering_shutdown()
