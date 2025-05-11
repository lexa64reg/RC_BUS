import pygame
import socket
import json
import time
import os
from decouple import config

#-------Настройки---------------
SERVER_IP = config('SERVER_IP',default='192.168.10.127')
SERVER_PORT = config('SERVER_PORT',default='5555')
STEERING_MAX_DEGREES = 100  # угол поворота руля
#--------------------------------

#------Инициализация----------------
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()
joysticks = []
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#--------------------------------

#---------------------------------
def normalize_axis(val):
    return int(val * 32767)

def axis_to_percent(val):
    #return int((1 - val) * 50) if val >= 0 else int((1 + abs(val)) * 50)
    return int((1 - abs(val)) * 50) if val <= 0 else int((1 + val) * 50)

def axis_to_degrees(val):
    return int(val * STEERING_MAX_DEGREES)
#----------------------------------

run = True
while run:
    pygame.event.pump()
    
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
          joy = pygame.joystick.Joystick(event.device_index)
          joysticks.append(joy)
        if event.type == pygame.QUIT:
            run = False
            
    for joystick in joysticks:
        raw_steer = joystick.get_axis(0)
        raw_throttle = -joystick.get_axis(2)
        raw_brake = -joystick.get_axis(3)

        axes = {
            #'steer': normalize_axis(raw_steer),
            #'throttle': normalize_axis(raw_throttle),
            #'brake': normalize_axis(raw_brake)
            'steer': axis_to_degrees(raw_steer),
            'throttle': axis_to_percent(raw_throttle),
            'brake': axis_to_percent(raw_brake)
        }

        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        payload = {
            'axes': axes,
            'buttons': buttons
        }

        # Отправка
        sock.sendto(json.dumps(payload).encode(), (SERVER_IP, SERVER_PORT))

        # Логирование
        steer_deg = axis_to_degrees(raw_steer)
        throttle_pct = axis_to_percent(raw_throttle)
        brake_pct = axis_to_percent(raw_brake)
        pressed_buttons = [i for i, b in enumerate(buttons) if b]
        print(f"[G923] Steering: {steer_deg:+4d}° | Throttle: {throttle_pct:3d}% | Brake: {brake_pct:3d}% | Buttons: {pressed_buttons}  {raw_throttle}")

        time.sleep(0.01)
        
        
pygame.quit()
