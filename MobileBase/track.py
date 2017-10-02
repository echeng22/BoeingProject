#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import numpy as np
import math
import sys
import tf


class trackControl(object):

    def __init__(self, track):

        self._trackPub = rospy.Publisher("vel", Twist, queue_size = 1)
        self.markerPub = rospy.Publisher("visualization_marker", Marker, queue_size = 1)
        self.hz = 100
        self.Rate = rospy.Rate(self.hz)
        self.twist_mag = Twist()
        self.marker_msg = Marker()
        self.marker_id = 0
        self.T = 1.0 / self.hz
        self.r = 1
        self.pos_x = self.r
        self.pos_y = 0
        self.num = 5000
        self.track = track
        self.br = tf.TransformBroadcaster() 
        self.trajectory()        

    def trajectory(self):

        # Circle, counterclockwise
        if(self.track == "-C"):
            for t in range(1, self.num + 1):
                theta = t * 2 * np.pi / self.num
                yp = self.r * math.sin(theta)
                xp = self.r * math.cos(theta) - self.r
                self.zero()
                self.control(xp, yp)
                self.pos_x = xp
                self.pos_y = yp

        # Square, counterclockwise, the radius is half of each side (self.num must be multiplicable by 4), assume self.pos_y = 0 and self.pos_x = self.r
        elif(self.track == "-S"):
            for t in range(1, (self.num / 4) + 1): # Straight Up
                yp = self.pos_y + ((self.r * 2.0) / (self.num / 4.0))
                self.zero()
                self.control(self.pos_x, yp)
                self.pos_y = yp
            for t in range(1, (self.num / 4) + 1): # Straight Left
                xp = self.pos_x - ((self.r * 2.0) / (self.num / 4.0))
                self.zero()
                self.control(xp, self.pos_y)
                self.pos_x = xp
            for t in range(1, (self.num / 4) + 1): # Straight Down
                yp = self.pos_y - ((self.r * 2.0) / (self.num / 4.0))
                self.zero()
                self.control(self.pos_x, yp)
                self.pos_y = yp
            for t in range(1, (self.num / 4) + 1): # Straight Right
                xp = self.pos_x + ((self.r * 2.0) / (self.num / 4.0))
                self.zero()
                self.control(xp, self.pos_y)
                self.pos_x = xp

        # Triangle, counterclockwise, the radius is half of the side (self.num must be multiplicable by 3), assume self.pos_y = 0 and self.pos_x = self.r
        elif (self.track == "-T"):
            self.num = 5001
            for t in range(1, (self.num / 3) + 1): # Bottom Right to Top vortex
                yp = self.pos_y + ((self.r * 2.0) / (self.num / 3.0)) * np.sin(np.pi / 3)
                xp = self.pos_x - ((self.r * 2.0) / (self.num / 3.0)) * np.cos(np.pi / 3)
                self.zero()
                self.control(xp, yp)  
                self.pos_x = xp
                self.pos_y = yp
            for t in range(1, (self.num / 3) + 1): # Top vortex to Bottom Right 
                yp = self.pos_y - ((self.r * 2.0) / (self.num / 3.0)) * np.cos(np.pi / 6)
                xp = self.pos_x - ((self.r * 2.0) / (self.num / 3.0)) * np.sin(np.pi / 6)
                self.zero()
                self.control(xp, yp)  
                self.pos_x = xp
                self.pos_y = yp
            for t in range(1, (self.num / 3) + 1): # Bottom Left to Bottom Right
                yp = self.pos_y
                xp = self.pos_x + ((self.r * 2.0) / (self.num / 3.0))
                self.zero()
                self.control(xp, yp)  
                self.pos_x = xp
                self.pos_y = yp

        while(1):
            self.zero()

    def control(self, x, y):
        # desired point (x, y)

        # Initial Twist message
        self.twist_mag.linear.x = 0
        self.twist_mag.linear.y = 0
        self.twist_mag.angular.z = 0

        # Calculate the velocity
        det_x = x - self.pos_x
        det_y = y - self.pos_y
        self.twist_mag.linear.x = det_x / self.T
        self.twist_mag.linear.y = det_y / self.T

        self._trackPub.publish(self.twist_mag)
        self.Rate.sleep()
        self.br.sendTransform((self.pos_x, self.pos_y, 0), (0, 0, 0, 1), rospy.Time.now(), "base", "map")
        self.marker_msg.header.frame_id = "/map"
        self.marker_msg.header.stamp = rospy.Time.now()
        self.marker_msg.ns = "trace"
        # if (self.marker_id < 2000):
        self.marker_id += 1
        # else :
        #     self.marker_id = 0
        self.marker_msg.id = self.marker_id
        self.marker_msg.type = 1
        self.marker_msg.action = 0
        self.marker_msg.pose.position.x = self.pos_x
        self.marker_msg.pose.position.y = self.pos_y
        self.marker_msg.scale.x = 0.01
        self.marker_msg.scale.y = 0.01
        self.marker_msg.scale.z = 0.01
        self.marker_msg.color.r = 0
        self.marker_msg.color.g = 1
        self.marker_msg.color.b = 0
        self.marker_msg.color.a = 1
        self.marker_msg.lifetime = rospy.Duration(30)
        self.markerPub.publish(self.marker_msg)

    def zero(self):

        # set Twist information to zero
        self.twist_mag.linear.x = 0
        self.twist_mag.linear.y = 0
        self.twist_mag.angular.z = 0

        self._trackPub.publish(self.twist_mag)
        self.Rate.sleep()

def main():
    rospy.init_node('track_control')
    if(len(sys.argv) <= 1):
        print "Please type arg to specify a type of track \n-C [Circle]\n-S [Square]\n-T [Triangle]"
    else:
        print "Arg: ", sys.argv[1]
        if (sys.argv[1] == "-C" or sys.argv[1] == "-S" or sys.argv[1] == "-T"):
            track = trackControl(sys.argv[1])
        # elif (sys.argv[1] == "-S"):
        else:
            print "Please type arg to specify a type of track \n-C [Circle]\n-S [Square]\n-T [Triangle]"
    rospy.spin()

if __name__=="__main__":
    main()

