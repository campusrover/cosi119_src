#!/usr/bin/env python3

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion


class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher("my_odom", Point, queue_size=1)
        self.old_pose = None
        self.dist = 0.0
        self.turn = 0.0
        print("My odom started")

    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        self.cur_pose = msg.pose.pose
        self.update_dist(self.cur_pose.position)
        self.update_turn(self.cur_pose.orientation)
        self.publish_data()
        self.old_pose = self.cur_pose

    def update_dist(self, position):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        pass
    
    def update_turn(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.turn` to current heading of robot.
        """
        pass

    def yaw_from_orientation(self, cur_orientation):
        pass

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object we create below is not used as a geometric point,
        # but simply as a data container for `self.dist` and `self.yaw` so we can
        # publish it on `my_odom`.
        print(f"{self.dist:0.3f}, {self.turn:0.3f}")
        data = Point()
        data.x = self.dist
        data.y = self.turn
        self.my_odom_pub.publish(data)


if __name__ == "__main__":
    rospy.init_node("my_odom")
    MyOdom()
    rospy.spin()
