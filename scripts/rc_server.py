#!/usr/bin/env python3
import rospy
import json
import socket
import threading
import RPi.GPIO as GPIO     #https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rc_forklift.forklift_driver import ForkliftDriver
from rc_forklift.perimeter import Perimeter
from rc_forklift.control_register import ControlRegister
import rc_forklift.control_register as reg
GPIO.setmode(GPIO.BOARD)



PORT = 6000

command = {
    'drive': 'stop',
    'steer': 'center',
    'lift': 'stop'
}

def stop():
    forklift.drive_stop()
    forklift.steer_center()
    forklift.lift_stop()


def callback(data):
    global command
    lock.acquire()
    command = json.loads(data.data)
    lock.release()


def main():
    rospy.init_node('forklift_server', anonymous=True)
    rospy.Subscriber('controls', String, callback)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    stop()
    while not perimeter.is_ready(): pass
    print("Server ready")
    while not rospy.is_shutdown():      
        if command['drive'] == 'forward':
            if perimeter.is_trespassing():
                 command['drive'] = 'stop'
            else:
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




    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #     s.bind(("", PORT))
    #     s.listen()
    #     while not rospy.is_shutdown():
    #         conn, addr = s.accept()
    #         with conn:
    #             print(f"Connected by {addr}")
    #             while True:
    #                 data = conn.recv(1024)
    #                 if not data: break
    #                 print(data)
    #                 register.set_reg(data)

    #                 drive = register.get_drive()
    #                 if drive == reg.FWD:
    #                     # if perimeter.is_trespassing():
    #                     #     command['drive'] = 'stop'
    #                     # else:
    #                     forklift.drive_forward()
    #                 elif drive == reg.REV:
    #                     forklift.drive_reverse()
    #                 else:
    #                     forklift.drive_stop()

    #                 steer = register.get_steer()
    #                 if steer == reg.LEFT:
    #                     forklift.steer_left()
    #                 elif steer == reg.RIGHT:
    #                     forklift.steer_right()
    #                 else:
    #                     forklift.steer_center()

    #                 lift = register.get_lift()
    #                 if lift == reg.UP:
    #                     forklift.lift_up()
    #                 elif lift == reg.DOWN:
    #                     forklift.lift_down()
    #                 else:
    #                     forklift.lift_stop()

    #             stop()
    #             print("Disconnected by peer")
        


if __name__ == '__main__':
    forklift = ForkliftDriver()
    perimeter = Perimeter()
    register = ControlRegister()
    lock = threading.Lock()
    try:
        stop()
        main()
    finally:
        stop()
