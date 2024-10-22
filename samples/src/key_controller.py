#!/usr/bin/env python

import rospy
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class key_controller:
    '''
    Key controller subscribes to the key publisher and publishes commands to the "hider"
    robot initialized in followbots.launch. Each function is mapped to a key in the dictionary
    in the constructor for this class.
    '''
    def __init__(self, botname):
        botname = "/" + botname
        rospy.init_node('key_controller')
        self.movement_dict = {
                ' ':self.halt,
                'a':self.rotate_left,
                'd':self.rotate_right,
                'w':self.move_forward,
                's':self.move_backward,
                'q':self.tilt_left,
                'e':self.tilt_right }
        self.key_sub = rospy.Subscriber('/keys', String, self.key_cb)
        self.move = rospy.Publisher(botname + '/cmd_vel', Twist, queue_size = 10)
        self.state = ' '
        self.twist = Twist()

    def key_cb(self, msg):
        self.state = msg.data
    
    def halt(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.move.publish(self.twist)

    def rotate_left(self):
        self.twist.linear.x = 0
        self.twist.angular.z = math.pi/3
        self.move.publish(self.twist)

    def rotate_right(self):
        self.twist.linear.x = 0
        self.twist.angular.z = -math.pi/3
        self.move.publish(self.twist)

    def move_forward(self):
        self.twist.angular.z = 0
        self.twist.linear.x = .3
        self.move.publish(self.twist)

    def move_backward(self):
        self.twist.angular.z = 0
        self.twist.linear.x = -.3
        self.move.publish(self.twist)

    def tilt_left(self):
        self.twist.angular.z = math.pi/3
        self.move.publish(self.twist)

    def tilt_right(self):
        self.twist.angular.z = -math.pi/3
        self.move.publish(self.twist)
    
    def move_cmd(self):
        state = self.state
        if state in self.movement_dict:
            self.movement_dict[state.lower()]()


'''
Subscribes to key commands and publishes velocity
commands at a rate of 60 times per second
'''
if __name__ == "__main__":
    ct = key_controller('hider')
    while not rospy.is_shutdown():
        ct.move_cmd()
        rospy.sleep(0.017)
