#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    # Print a message to indicate that the callback function is called
    print("Received laser scan message")
    
    # Extract data from the laser scan message and process it
    regions = [ 
      min(msg.ranges[0:143]),
      min(msg.ranges[144:287]),
      min(msg.ranges[288:431]),
      min(msg.ranges[432:575]),
      min(msg.ranges[576:713]),
     ]
    
    # Log the processed data
    rospy.loginfo(regions)

def main():
    rospy.init_node('reading_laser')
    sub = rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
