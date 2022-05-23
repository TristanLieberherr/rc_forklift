#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO     #https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root
from smbus2 import SMBus
from std_msgs.msg import String
GPIO.setmode(GPIO.BOARD)



class DCMotor:
    def __init__(self, pin1, pin2):
        self._pin1 = pin1
        self._pin2 = pin2
        GPIO.setup(self._pin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._pin2, GPIO.OUT, initial=GPIO.LOW)
    
    def stop(self):
        GPIO.output(self._pin1, GPIO.LOW)
        GPIO.output(self._pin2, GPIO.LOW)

    def forward(self):
        GPIO.output(self._pin1, GPIO.LOW)
        GPIO.output(self._pin2, GPIO.HIGH)

    def reverse(self):
        GPIO.output(self._pin1, GPIO.HIGH)
        GPIO.output(self._pin2, GPIO.LOW)



class ServoMotor:
    def __init__(self, bus, channel):
        self._bus = bus
        self._channel = channel
        self.__CENTER = 315
        self.__STEP = 55
        self.__I2C_ADDRESS         = 0x7F
        self.__MODE1               = 0x00
        self.__PRESCALE            = 0xFE
        self.__CHANNEL_ON_L        = 0x06
        self.__CHANNEL_ON_H        = 0x07
        self.__CHANNEL_OFF_L       = 0x08
        self.__CHANNEL_OFF_H       = 0x09
        self.__SLEEP               = 0x10
        self.__OSC_CLOCK           = 25000000.0 #25 MHz
        self.set_frequency(50)

    def set_pwm(self, start, end):
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_ON_L + 4 * self._channel, start & 0xFF)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_ON_H + 4 * self._channel, start >> 8)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_OFF_L + 4 * self._channel, end & 0xFF)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_OFF_H + 4 * self._channel, end >> 8)

    def set_frequency(self, frequency):
        prescale_value = int(round(self.__OSC_CLOCK / (4096 * frequency)) - 1)
        mode1 = self._bus.read_byte_data(self.__I2C_ADDRESS, self.__MODE1)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__MODE1, mode1 | self.__SLEEP)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__PRESCALE, prescale_value)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__MODE1, mode1 & ~self.__SLEEP)

    def center(self):
        self.set_pwm(0, self.__CENTER)

    def right(self):
        self.set_pwm(0, self.__CENTER - self.__STEP)

    def left(self):
        self.set_pwm(0, self.__CENTER + self.__STEP)



class Forklift:
    def __init__(self):
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

    def drive_forward(self):
        self._driving_motor.forward()

    def drive_reverse(self):
        self._driving_motor.reverse()

    def drive_stop(self):
        self._driving_motor.stop()

    def steer_center(self):
        self._steering_motor.center()

    def steer_right(self):
        self._steering_motor.right()

    def steer_left(self):
        self._steering_motor.left()

    def lift_up(self):
        self._lifting_motor.forward()
        
    def lift_down(self):
        self._lifting_motor.reverse()

    def lift_stop(self):
        self._lifting_motor.stop()



DRIVE_STOP      = "DRIVE_STOP"
DRIVE_FORWARD   = "DRIVE_FORWARD"
DRIVE_REVERSE   = "DRIVE_REVERSE"
STEER_CENTER    = "STEER_CENTER"
STEER_RIGHT     = "STEER_RIGHT"
STEER_LEFT      = "STEER_LEFT"
LIFT_STOP       = "LIFT_STOP"
LIFT_UP         = "LIFT_UP"
LIFT_DOWN       = "LIFT_DOWN"



def callback(data):
    control = data.data
    #print(control)
    if control == DRIVE_STOP:
        forklift.drive_stop()
    elif control == DRIVE_FORWARD:
        forklift.drive_forward()
    elif control == DRIVE_REVERSE:
        forklift.drive_reverse()
    elif control == STEER_CENTER:
        forklift.steer_center()
    elif control == STEER_RIGHT:
        forklift.steer_right()
    elif control == STEER_LEFT:
        forklift.steer_left()
    elif control == LIFT_STOP:
        forklift.lift_stop()
    elif control == LIFT_UP:
        forklift.lift_up()
    elif control == LIFT_DOWN:
        forklift.lift_down()



def listen():
    rospy.init_node('forklift_server', anonymous=True)
    rospy.Subscriber('controls', String, callback)
    print("Server ready")
    rospy.spin()

forklift = Forklift()

if __name__ == '__main__':
    listen()
