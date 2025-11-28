# motor_control.py
from gpiozero import Motor
from time import sleep

class MotorDriver:
    def __init__(self, left_pins, right_pins):
        self.left_motor = Motor(forward=left_pins[0], backward=left_pins[1],
                                enable=left_pins[2], pwm=True)
        self.right_motor = Motor(forward=right_pins[0], backward=right_pins[1],
                                 enable=right_pins[2], pwm=True)
        self.current_speed = 0.5
        self.is_forward = True
        self.stop_all()

    def move_forward(self, speed=None):
        speed = speed or self.current_speed
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)
        self.is_forward = True

    def move_backward(self, speed=None):
        speed = speed or self.current_speed
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)
        self.is_forward = False

    def stop_all(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def turn_left(self, speed=None):
        speed = speed or self.current_speed
        self.left_motor.backward(speed * 0.6)
        self.right_motor.forward(speed)

    def turn_right(self, speed=None):
        speed = speed or self.current_speed
        self.left_motor.forward(speed)
        self.right_motor.backward(speed * 0.6)

    def set_speed(self, level):
        """Set speed between 0.0–1.0"""
        self.current_speed = max(0.0, min(1.0, level))
        if self.is_forward:
            self.move_forward(self.current_speed)
        else:
            self.move_backward(self.current_speed)

    def close(self):
        self.stop_all()
        self.left_motor.close()
        self.right_motor.close()
