#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeSolverSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        raise NotImplementedError

    def follow_wall(self):
        """Makes the robot follow a wall."""
        raise NotImplementedError

if __name__ == '__main__':
    rospy.init_node('maze_solver_sim')
    MazeSolverSim().follow_wall()
