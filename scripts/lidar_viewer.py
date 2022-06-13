#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from sensor_msgs.msg import LaserScan
from rc_forklift.perimeter import Perimeter, FORKLIFT_THETAS, FORKLIFT_RANGES



display_angle_min = 210/180*np.pi
display_angle_max = 330/180*np.pi
display_range = 1
fig = plt.figure()
ax = fig.add_subplot(projection='polar')

def animate(i):
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
    ax.plot(FORKLIFT_THETAS, FORKLIFT_RANGES, 'y')

def main():
    rospy.init_node('lidar_viewer', anonymous=True)
    rospy.Subscriber('scan', LaserScan, perimeter.update)
    while not perimeter.is_ready(): pass
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()
    
    
if __name__ == '__main__':
    perimeter = Perimeter()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
