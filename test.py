from rplidar import RPLidar
from motor_control import MotorDriver
import time

# Setup
lidar = RPLidar('/dev/ttyUSB0')
lidar._set_pwm(500)
motors = MotorDriver(left_pins=(17, 22, 25), right_pins=(23, 24, 5))

OBSTACLE_DISTANCE = 400

def is_in_front(angle):
    return angle >= 330 or angle <= 30

try:
    for scan in lidar.iter_scans(max_buf_meas=2000):
        front_distances = [dist for (_, angle, dist) in scan if is_in_front(angle) and dist > 0]

        if front_distances:
            min_distance = min(front_distances)
            print(f"Closest obstacle ahead: {min_distance:.1f} mm")

            if min_distance < OBSTACLE_DISTANCE:
                print("⚠️ Obstacle detected! Stopping & turning...")
                motors.stop_all()
                time.sleep(0.5)
                motors.turn_right(0.5)
                time.sleep(1.0)
                motors.stop_all()
            else:
                motors.move_forward(0.6)
except KeyboardInterrupt:
    print("Stopping...")
finally:
    motors.close()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
