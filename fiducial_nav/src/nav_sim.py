#!/usr/bin/env python3

import math
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

class NavSim:
    def __init__(self):
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
       
    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        raise NotImplementedError

    def turn_to_heading(self, target_yaw, base_vel):
        """
        Turns the robot to heading `target_yaw` with a base velocity of
        `base_vel`.
        """
        raise NotImplementedError
        
    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node
        does the actual mapping.
        """
        raise NotImplementedError

    def match_pin_rotation(self, pin_id):
        """
        Rotates the robot so that its `base_link` frame's orientation matches
        that of the target pin's frame.
        """
        raise NotImplementedError
        
    def face_pin(self, pin_id):
        """
        Rotates the robot so that it faces the target pin.
        """
        raise NotImplementedError

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """
        raise NotImplementedError

if __name__ == '__main__':
    rospy.init_node('nav_sim')

    nav = NavSim()
    nav.scan_for_fids()
    target_pin_ids = [0, 1, 2, 3]
    for pin_id in target_pin_ids:
        nav.match_pin_rotation(pin_id)
        nav.face_pin(pin_id)
        nav.move_to_pin(pin_id)