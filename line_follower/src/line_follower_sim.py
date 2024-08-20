#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowerSim:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def image_callback(self, msg):
        """Callback to `self.image_sub`."""
        raise NotImplementedError
    
    def follow_line(self):
        """Follow a yellow line."""
        raise NotImplementedError

    def run(self):
        """Run the Program."""
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            self.follow_line()

if __name__ == '__main__':
    rospy.init_node('line_follower_sim')
    LineFollowerSim().run()