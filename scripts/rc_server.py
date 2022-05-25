#!/usr/bin/env python3
import rospy
import json
import threading
import RPi.GPIO as GPIO     #https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rc_forklift.forklift_driver import ForkliftDriver
from rc_forklift.perimeter import Perimeter
GPIO.setmode(GPIO.BOARD)



command = {
    'drive': 'stop',
    'steer': 'center',
    'lift': 'stop'
}


def callback(data):
    global command
    lock.acquire()
    command = json.loads(data.data)
    lock.release()


def main():
    rospy.init_node('forklift_server', anonymous=True)
    rospy.Subscriber('controls', String, callback)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    print("Server ready")


    while not rospy.is_shutdown():
        if perimeter.is_trespassing():
            forklift.drive_stop()
        elif command['drive'] == 'forward':
            forklift.drive_forward()
        elif command['drive'] == 'reverse':
            forklift.drive_reverse()
        else:
            forklift.drive_stop()

        if command['steer'] == 'left':
            forklift.steer_left()
        elif command['steer'] == 'right':
            forklift.steer_right()
        else:
            forklift.steer_center()

        if command['lift'] == 'up':
            forklift.lift_up()
        elif command['lift'] == 'down':
            forklift.lift_down()
        else:
            forklift.lift_stop()

        
        # lock.acquire()
        # command['drive'] = 'stop'
        # command['steer'] = 'center'
        # command['lift'] = 'stop'
        # lock.release()
        # sleep(0.1)


if __name__ == '__main__':
    forklift = ForkliftDriver()
    perimeter = Perimeter()
    lock = threading.Lock()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        forklift.drive_stop()
        forklift.steer_center()
        forklift.lift_stop()
