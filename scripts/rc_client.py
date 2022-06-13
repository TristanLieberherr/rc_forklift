#!/usr/bin/env python
import rospy
import pygame
import socket
from sensor_msgs.msg import LaserScan
from rc_forklift.perimeter import Perimeter
from rc_forklift.keyboard_controller import KeyboardController
from rc_forklift.control_register import ControlRegister
import rc_forklift.control_register as reg



HOST = "forklift"
PORT = 6000

def main():
    rospy.init_node('forklift_client', anonymous=True)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    pygame.init()
    display = pygame.display.set_mode((300, 300))
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print("Connected to {host}:{port}".format(host=HOST, port=PORT))
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT: exit()
                keyboard.update(event)

                if keyboard.is_pressed(pygame.K_w) and not keyboard.is_pressed(pygame.K_s):
                    register.set_drive(reg.FWD)
                elif not keyboard.is_pressed(pygame.K_w) and keyboard.is_pressed(pygame.K_s):
                    register.set_drive(reg.REV)
                else:
                    register.set_drive(reg.STOP)

                if keyboard.is_pressed(pygame.K_a) and not keyboard.is_pressed(pygame.K_d):
                    register.set_steer(reg.LEFT)
                elif not keyboard.is_pressed(pygame.K_a) and keyboard.is_pressed(pygame.K_d):
                    register.set_steer(reg.RIGHT)
                else:
                    register.set_steer(reg.CENTER)

                if keyboard.is_pressed(pygame.K_UP) and not keyboard.is_pressed(pygame.K_DOWN):
                    register.set_lift(reg.UP)
                elif not keyboard.is_pressed(pygame.K_UP) and keyboard.is_pressed(pygame.K_DOWN):
                    register.set_lift(reg.DOWN)
                else:
                    register.set_lift(reg.STOP)
            s.send(register.get_reg().to_bytes(1, 'big'))


if __name__ == '__main__':
    keyboard = KeyboardController()
    perimeter = Perimeter()
    register = ControlRegister()
    try:
        main()
    finally: 
        pygame.quit()
