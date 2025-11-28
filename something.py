from motor_control import MotorDriver
from time import sleep

motors = MotorDriver(
    left_pins=(17, 22, 25),   # forward, backward, enable
    right_pins=(23, 24, 5)
)

motors.move_forward(0.5)
sleep(5)
motors.turn_left()
sleep(1)
motors.stop_all()
motors.close()
