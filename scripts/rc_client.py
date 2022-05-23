#!/usr/bin/env python3
import rospy
import sys
import threading
import pygame
import math
import matplotlib.pyplot as plt
import numpy as np
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

display_angle_min = 210/180*np.pi
display_angle_max = 330/180*np.pi
display_range = 1
forklift_front = 0.25
forklift_back = 0.12
forklift_side = 0.08
forklift_x = np.array([-forklift_side, -forklift_side, forklift_side, forklift_side, -forklift_side])
forklift_y = np.array([-forklift_back, forklift_front, forklift_front, -forklift_back, -forklift_back])
forklift_thetas = np.arctan2(forklift_y, forklift_x)
forklift_ranges = np.sqrt(forklift_x**2+forklift_y**2)
fig = plt.figure()
ax = fig.add_subplot(projection='polar')
keyboard = KeyboardController()
perimeter = Perimeter()
perimeter.create_segment([-0.2, 0.2], [-0.2, 0.4])
perimeter.create_segment([-0.2, 0.4], [0.2, 0.4])
perimeter.create_segment([0.2, 0.2], [0.2, 0.4])


def animate():
    ax.clear()
    ax.set_ylim(0, display_range)
    thetas = perimeter.get_thetas()
    perimeter.get_lock().acquire()
    ax.plot(thetas, perimeter.get_scan_ranges(), ',r')
    color = 'r' if perimeter.is_trespassing() else 'g'
    ax.plot(thetas, perimeter.get_ranges(), color=color, linewidth='0.5')
    perimeter.get_lock().release()
    ax.plot([display_angle_min]*2, [0, display_range], 'b', linewidth='0.5')
    ax.plot([display_angle_max]*2, [0, display_range], 'b', linewidth='0.5')
    ax.plot(forklift_thetas, forklift_ranges, 'y')
    plt.draw()
    plt.pause(0.0001)

def main():
    rospy.init_node('forklift_client', anonymous=True)
    pub = rospy.Publisher('controls', String, queue_size=10)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    plt.show(block=False)
    while not rospy.is_shutdown():
        animate()
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
