#!/usr/bin/env python3
import time
import signal
import numpy as np
import matplotlib.pyplot as plt

# ======= MOTOR IMPORT =======
from gpiozero import Motor, Device
from gpiozero.pins.rpigpio import RPiGPIOFactory
Device.pin_factory = RPiGPIOFactory()  # Force RPiGPIO backend

# ======= LIDAR IMPORT =======
from rplidar import RPLidar

# ================= CONFIG =================
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_PWM = 500           # Spin speed
MAX_BUF = 2000            # Larger buffer
SHOW_PLOT = True          # Set False to disable live plot

OBSTACLE_DISTANCE = 400   # mm threshold for front obstacles
FRONT_ANGLE_RANGE = (330, 30)  # degrees (0° = front)

# Motor pins: (forward, backward, enable)
LEFT_PINS = (17, 22, 25)
RIGHT_PINS = (23, 24, 5)

# Motor speed
FORWARD_SPEED = 0.5
TURN_SPEED = 0.5
TURN_TIME = 1.0  # seconds to turn when avoiding

# ===========================================

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\nStopping...")

signal.signal(signal.SIGINT, signal_handler)

# ====== INITIALIZE MOTORS ======
left_motor = Motor(forward=LEFT_PINS[0], backward=LEFT_PINS[1], enable=LEFT_PINS[2])
right_motor = Motor(forward=RIGHT_PINS[0], backward=RIGHT_PINS[1], enable=RIGHT_PINS[2])

def move_forward(speed=FORWARD_SPEED):
    left_motor.forward(speed)
    right_motor.forward(speed)

def stop_all():
    left_motor.stop()
    right_motor.stop()

def turn_right(speed=TURN_SPEED):
    left_motor.forward(speed)
    right_motor.backward(speed)

def turn_left(speed=TURN_SPEED):
    left_motor.backward(speed)
    right_motor.forward(speed)

def close_motors():
    stop_all()
    left_motor.close()
    right_motor.close()

# ====== INITIALIZE LIDAR ======
lidar = RPLidar(LIDAR_PORT)
lidar._set_pwm(LIDAR_PWM)
time.sleep(1)

# ====== SETUP PLOT ======
if SHOW_PLOT:
    plt.ion()
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='polar')
    scan_plot, = ax.plot([], [], 'g.', markersize=2)
    ax.set_rmax(4000)
    ax.grid(True)
    ax.set_title("RPLidar A1 - Live Scan", va='bottom')

def in_front(angle):
    """Return True if angle is in front sector"""
    return angle >= FRONT_ANGLE_RANGE[0] or angle <= FRONT_ANGLE_RANGE[1]

try:
    print("Starting autonomous robot. Press Ctrl+C to stop.")
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=MAX_BUF)):
        distances = []
        angles = []

        # Check front distances for obstacle
        front_distances = []
        for _, angle, distance in scan:
            if distance == 0:
                continue
            distances.append(distance)
            angles.append(np.deg2rad(angle))
            if in_front(angle):
                front_distances.append(distance)

        # Display scan info
        print(f"Scan {i}: {len(scan)} points | Closest front: {min(front_distances, default=0):.0f} mm")

        # Update plot
        if SHOW_PLOT:
            scan_plot.set_data(angles, distances)
            plt.pause(0.001)

        # Check obstacle
        if front_distances and min(front_distances) < OBSTACLE_DISTANCE:
            print("⚠️ Obstacle detected! Stopping & turning...")
            stop_all()
            time.sleep(0.3)
            turn_right()
            time.sleep(TURN_TIME)
            stop_all()
            time.sleep(0.2)
        else:
            move_forward(FORWARD_SPEED)

        if not running:
            break

except Exception as e:
    print("Error:", e)

finally:
    print("Stopping robot and disconnecting LiDAR...")
    stop_all()
    close_motors()
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except:
        pass
    print("Robot stopped cleanly.")
