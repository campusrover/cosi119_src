#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point

class LineFollowerReal:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.init_states()

    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }

    def my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        raise NotImplementedError

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        # Use the cv_bridge package to convert ROS sensor_msgs/Image messages 
        # into OpenCV2 images (cf. PRR p.197)
        raise NotImplementedError

    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        raise NotImplementedError
  
    def follow_line(self):
        """Follow a black line."""
        raise NotImplementedError

    def avoid_obstacle(self):
        """Avoid an obstacle of known dimensions."""
        raise NotImplementedError
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.states['following_line']:
                self.follow_line()
            elif self.states['avoiding_obstacle']:
                self.avoid_obstacle()
            rate.sleep()
           
if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
