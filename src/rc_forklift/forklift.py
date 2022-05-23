import RPi.GPIO as GPIO
from smbus2 import SMBus
from rc_forklift.dc_motor import DCMotor
from rc_forklift.servo_motor import ServoMotor



class Forklift:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        self._bus = SMBus(1)
        self._driving_motor = DCMotor(35, 37)
        self._lifting_motor = DCMotor(31, 33)
        self._steering_motor = ServoMotor(self._bus, 0)
        self._reset_motors()

    def _dispose(self):
        self._reset_motors()
        GPIO.cleanup()
        self._bus.close()

    def _reset_motors(self):
        self._driving_motor.stop()
        self._steering_motor.center()

    def forward(self):
        self._driving_motor.forward()

    def reverse(self):
        self._driving_motor.reverse()

    def stop(self):
        self._driving_motor.stop()

    def center(self):
        self._steering_motor.center()

    def right(self):
        self._steering_motor.right()

    def left(self):
        self._steering_motor.left()

    def up(self):
        self._lifting_motor.forward()
        
    def down(self):
        self._lifting_motor.reverse()

    def lift_stop(self):
        self._lifting_motor.stop()
