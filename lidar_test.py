#!/usr/bin/env python3
"""
Improved LiDAR Test Script
Uses LidarHandler for better buffer management
"""

from lidar_handler import LidarHandler
import matplotlib.pyplot as plt
import numpy as np
import signal
import time

# ==== CONFIG ====
PORT_NAME = '/dev/ttyUSB0'
PWM_SPEED = 500
MAX_BUF = 3600     # Increased buffer size
SHOW_PLOT = True
# =================

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\nStopping...")

signal.signal(signal.SIGINT, signal_handler)

print("Starting LiDAR with improved buffer handling...")
print("Press Ctrl+C to stop")

# Initialize LiDAR handler
lidar = LidarHandler(port=PORT_NAME, pwm_speed=PWM_SPEED, max_buffer=MAX_BUF)
lidar.start()

# Optional live plot
if SHOW_PLOT:
    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='polar')
    scan_plot, = ax.plot([], [], 'g.', markersize=2)
    ax.set_rmax(4000)
    ax.grid(True)
    ax.set_title("RPLidar A1 - Live Scan (Improved Buffer)", va='bottom')

try:
    scan_count = 0
    while running:
        scan = lidar.get_latest_scan()
        
        if scan is not None and scan['count'] > 0:
            distances = scan['distances']
            angles = scan['angles']
            
            print(f"Scan {scan_count}: {scan['count']} points | "
                  f"Min: {np.min(distances):.0f} mm | "
                  f"Max: {np.max(distances):.0f} mm")

            if SHOW_PLOT:
                angles_rad = np.deg2rad(angles)
                scan_plot.set_data(angles_rad, distances)
                plt.pause(0.01)
            
            scan_count += 1
        
        time.sleep(0.1)

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

finally:
    print("Stopping LiDAR...")
    lidar.stop()
    print("LiDAR stopped cleanly.")
