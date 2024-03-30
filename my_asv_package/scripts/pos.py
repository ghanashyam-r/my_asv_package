#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(data):
    # Extract position coordinates
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    # Display position coordinates
    rospy.loginfo("ASV Position - X: %f, Y: %f, Z: %f", x, y, z)

def listener():
    rospy.init_node('asv_position_display')
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
