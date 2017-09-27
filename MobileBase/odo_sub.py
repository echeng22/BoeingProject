#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import numpy as np


def callback(data):

    # position
    pose_x = data.position.x
    print pose_x
    pose_y = data.position.y
    print pose_y
    # orientation
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    pose_theta = euler[2]
    print pose_theta


def main():
    rospy.init_node('odometry_sub')
    rospy.Subscriber("/odom", Pose, callback)
    rospy.spin()

if __name__=="__main__":
    main()
