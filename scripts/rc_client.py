#!/usr/bin/env python3
import rospy
import sys
import pygame
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rc_forklift.perimeter import Perimeter



class KeyboardController:
    def __init__(self):
        self.keys = {pygame.K_w: False, pygame.K_a: False, pygame.K_s: False, pygame.K_d: False, pygame.K_UP: False, pygame.K_DOWN: False}

    def update(self, event):
        self.event = event
        if self.event.type == pygame.KEYDOWN:
            self.keys[self.event.key] = True
        elif self.event.type == pygame.KEYUP:
            self.keys[self.event.key] = False

    def is_pressed(self, key) -> bool:
        return self.keys[key]

DRIVE_STOP      = "DRIVE_STOP"
DRIVE_FORWARD   = "DRIVE_FORWARD"
DRIVE_REVERSE   = "DRIVE_REVERSE"
STEER_CENTER    = "STEER_CENTER"
STEER_RIGHT     = "STEER_RIGHT"
STEER_LEFT      = "STEER_LEFT"
LIFT_STOP       = "LIFT_STOP"
LIFT_UP         = "LIFT_UP"
LIFT_DOWN       = "LIFT_DOWN"

keyboard = KeyboardController()
perimeter = Perimeter()

def main():
    rospy.init_node('forklift_client', anonymous=True)
    pub = rospy.Publisher('controls', String, queue_size=10)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            keyboard.update(event)
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if keyboard.is_pressed(pygame.K_w) and not keyboard.is_pressed(pygame.K_s):
                if perimeter.is_trespassing():
                    pub.publish(DRIVE_STOP)
                else:
                    pub.publish(DRIVE_FORWARD)
            elif not keyboard.is_pressed(pygame.K_w) and keyboard.is_pressed(pygame.K_s):
                pub.publish(DRIVE_REVERSE)
            else:
                pub.publish(DRIVE_STOP)

            if keyboard.is_pressed(pygame.K_a) and not keyboard.is_pressed(pygame.K_d):
                pub.publish(STEER_LEFT)
            elif not keyboard.is_pressed(pygame.K_a) and keyboard.is_pressed(pygame.K_d):
                pub.publish(STEER_RIGHT)
            else:
                pub.publish(STEER_CENTER)

            if keyboard.is_pressed(pygame.K_UP) and not keyboard.is_pressed(pygame.K_DOWN):
                pub.publish(LIFT_UP)
            elif not keyboard.is_pressed(pygame.K_UP) and keyboard.is_pressed(pygame.K_DOWN):
                pub.publish(LIFT_DOWN)
            else:
                pub.publish(LIFT_STOP)

if __name__ == '__main__':
    try:
        pygame.init()
        display = pygame.display.set_mode((300, 300))
        main()
    except rospy.ROSInterruptException:
        pass
