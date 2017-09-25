#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import math


class trackControl(object):

    def __init__(self):

        self._trackPub = rospy.Publisher("vel", Twist, queue_size = 1)
        self.Rate = rospy.Rate(10)
        self.twist_mag = Twist()
        self.T = 10
        self.pos_x = 0
        self.pos_y = 0
        self.num = 50
        self.r = 10
        self.trajectory()

    def trajectory(self):

        # Circle, clockwise
        for t in range(1, self.num + 1):
            theta = t * 2 * 3.1415926 / self.num
            yp = self.r * math.sin(theta) - self.r
            xp = self.r * math.cos(theta)
            self.control(xp, yp)
            self.pos_x = xp
            self.pos_y = yp

    def control(self, x, y):
        # desired point (x, y)

        # Initial Twist massage
        self.twist_mag.linear.x = 0
        self.twist_mag.linear.y = 0
        self.twist_mag.angular.z = 0

        # Calculate the velocity
        det_x = x - self.pos_x
        det_y = y - self.pos_y
        # forward or backward
        if det_x == 0:
            self.twist_mag.linear.y = det_y / self.T
        # Turn left or right
        elif det_y == 0:
            self.twist_mag.linear.x = det_x / self.T
        # Go slash
        else:
            self.twist_mag.linear.x = det_x / self.T
            self.twist_mag.linear.y = det_y / self.T

        self._trackPub.publish(self.twist_mag)
        self.Rate.sleep()

def main():
    rospy.init_node('track_control', anonymous = True)
    track = trackControl()
    rospy.spin()

if __name__=="__main__":
    main()
