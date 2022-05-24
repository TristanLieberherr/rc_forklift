#!/usr/bin/env python3
import rospy
import sys
import pygame
import json
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

def main():
    rospy.init_node('forklift_client', anonymous=True)
    pub = rospy.Publisher('controls', String, queue_size=10)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    pygame.init()
    display = pygame.display.set_mode((300, 300))
    command = {
        'drive': 'stop',
        'steer': 'center',
        'lift': 'stop'
    }
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            keyboard.update(event)
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if keyboard.is_pressed(pygame.K_w) and not keyboard.is_pressed(pygame.K_s):
                command['drive'] = 'forward'
            elif not keyboard.is_pressed(pygame.K_w) and keyboard.is_pressed(pygame.K_s):
                command['drive'] = 'reverse'
            else:
                command['drive'] = 'stop'

            if keyboard.is_pressed(pygame.K_a) and not keyboard.is_pressed(pygame.K_d):
                command['steer'] = 'left'
            elif not keyboard.is_pressed(pygame.K_a) and keyboard.is_pressed(pygame.K_d):
                command['steer'] = 'right'
            else:
                command['steer'] = 'center'

            if keyboard.is_pressed(pygame.K_UP) and not keyboard.is_pressed(pygame.K_DOWN):
                command['lift'] = 'up'
            elif not keyboard.is_pressed(pygame.K_UP) and keyboard.is_pressed(pygame.K_DOWN):
                command['lift'] = 'down'
            else:
                command['lift'] = 'stop'
            pub.publish(json.dumps(command))


if __name__ == '__main__':
    keyboard = KeyboardController()
    perimeter = Perimeter()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
