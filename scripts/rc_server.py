#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO     #https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rc_forklift.forklift import Forklift
from rc_forklift.perimeter import Perimeter
GPIO.setmode(GPIO.BOARD)



DRIVE_STOP      = "DRIVE_STOP"
DRIVE_FORWARD   = "DRIVE_FORWARD"
DRIVE_REVERSE   = "DRIVE_REVERSE"
STEER_CENTER    = "STEER_CENTER"
STEER_RIGHT     = "STEER_RIGHT"
STEER_LEFT      = "STEER_LEFT"
LIFT_STOP       = "LIFT_STOP"
LIFT_UP         = "LIFT_UP"
LIFT_DOWN       = "LIFT_DOWN"

perimeter = Perimeter()

def callback(data):
    control = data.data
    #print(control)
    if control == DRIVE_STOP:
        forklift.drive_stop()
    elif control == DRIVE_FORWARD:
        if perimeter.is_trespassing():
            forklift.drive_stop()
        else:
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
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    print("Server ready")
    rospy.spin()

if __name__ == '__main__':
    forklift = Forklift()
    listen()
