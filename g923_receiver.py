import socket
import json

import smbus2
import time

# Настройки
LISTEN_PORT = 5555
STEERING_MAX_DEGREES = 450  # ±450°
AXIS_MAX = 32767

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))

AS5600_ADDR = 0x36
ANGLE_REG = 0x0E
bus = smbus2.SMBus(1)

def read_angle():
    # Read two bytes from the angle register
    raw_data = bus.read_i2c_block_data(AS5600_ADDR, ANGLE_REG, 2)
    angle = (raw_data[0] << 8) | raw_data[1]  # Combine MSB and LSB
    angle = angle & 0x0FFF  # Mask to 12 bits
    return (angle / 4096.0) * 360.0  # Convert to degrees



def axis_to_degrees(val):
    return int(val / AXIS_MAX * STEERING_MAX_DEGREES)

def axis_to_percent(val):
    return int(val / AXIS_MAX * 100)

print("[Receiver] G923 receiver started, waiting for data...")

while True:
    data, _ = sock.recvfrom(2048)
    try:
        state = json.loads(data)

        steer = state['axes']['steer']
        throttle = state['axes']['throttle']
        brake = state['axes']['brake']

        # Лог
        steer_deg = axis_to_degrees(steer)
        throttle_pct = axis_to_percent(throttle)
        brake_pct = axis_to_percent(brake)
        pressed_buttons = [i for i, b in enumerate(state['buttons']) if b]

        angle = read_angle()
        
        print(f"[G923] Steering: {steer_deg:+4d}° | Throttle: {throttle_pct:3d}% | Brake: {brake_pct:3d}% | Buttons: {pressed_buttons} | Angle: {angle:.2f} degrees")
        
        
        
    except Exception as e:
        print(f"[Error] {e}")
