#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_cb(msg):
    print(msg.ranges)

if __name__ == '__main__':
    rospy.init_node('topic_csv', log_level=rospy.DEBUG)
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
    rospy.spin()
