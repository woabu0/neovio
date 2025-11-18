#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gpiozero import Motor
import time

class MotorController:
    def __init__(self, left_pins, right_pins):
        # left_pins: (forward_pin, backward_pin, enable_pin)
        # right_pins: same
        self.left_motor = Motor(forward=left_pins[0], backward=left_pins[1], enable=left_pins[2], pwm=True)
        self.right_motor = Motor(forward=right_pins[0], backward=right_pins[1], enable=right_pins[2], pwm=True)
        self.max_speed = 0.8  # Adjust as needed

    def cmd_vel_callback(self, data):
        linear_x = data.linear.x
        angular_z = data.angular.z

        # Differential drive calculation
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        # Normalize speeds to [-1, 1]
        max_spd = max(abs(left_speed), abs(right_speed))
        if max_spd > 1.0:
            left_speed /= max_spd
            right_speed /= max_spd

        # Scale to max_speed
        left_speed *= self.max_speed
        right_speed *= self.max_speed

        # Control motors
        if left_speed > 0:
            self.left_motor.forward(left_speed)
        elif left_speed < 0:
            self.left_motor.backward(abs(left_speed))
        else:
            self.left_motor.stop()

        if right_speed > 0:
            self.right_motor.forward(right_speed)
        elif right_speed < 0:
            self.right_motor.backward(abs(right_speed))
        else:
            self.right_motor.stop()

    def cleanup(self):
        self.left_motor.close()
        self.right_motor.close()

if __name__ == '__main__':
    # GPIO pins - adjust based on your wiring
    LEFT_PINS = (17, 27, 22)  # (forward, backward, enable)
    RIGHT_PINS = (23, 24, 25)

    controller = MotorController(LEFT_PINS, RIGHT_PINS)
    rospy.init_node('motor_control_node')
    rospy.Subscriber('/cmd_vel', Twist, controller.cmd_vel_callback)
    rospy.on_shutdown(controller.cleanup)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
