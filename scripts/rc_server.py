#!/usr/bin/env python3
import rospy
import socket
import RPi.GPIO as GPIO     #https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root
from sensor_msgs.msg import LaserScan
from rc_forklift.forklift_driver import ForkliftDriver
from rc_forklift.perimeter import Perimeter
from rc_forklift.control_register import ControlRegister
import rc_forklift.control_register as reg
GPIO.setmode(GPIO.BOARD)



PORT = 6000

def stop():
    forklift.drive_stop()
    forklift.steer_center()
    forklift.lift_stop()

def main():
    rospy.init_node('forklift_server', anonymous=True)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", PORT))
        s.listen()
        while not rospy.is_shutdown():
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024)
                    if not data: break
                    print(data)
                    register.set_reg(int.from_bytes(data, 'big'))

                    drive = register.get_drive()
                    if drive == reg.FWD:
                        if perimeter.is_trespassing():
                            forklift.drive_stop()
                        else:
                            forklift.drive_forward()
                    elif drive == reg.REV:
                        forklift.drive_reverse()
                    else:
                        forklift.drive_stop()

                    steer = register.get_steer()
                    if steer == reg.LEFT:
                        forklift.steer_left()
                    elif steer == reg.RIGHT:
                        forklift.steer_right()
                    else:
                        forklift.steer_center()

                    lift = register.get_lift()
                    if lift == reg.UP:
                        forklift.lift_up()
                    elif lift == reg.DOWN:
                        forklift.lift_down()
                    else:
                        forklift.lift_stop()

                stop()
                print("Disconnected by peer")


if __name__ == '__main__':
    forklift = ForkliftDriver()
    perimeter = Perimeter()
    register = ControlRegister()
    try:
        stop()
        main()
    finally:
        stop()
