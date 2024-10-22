#!/usr/bin/env python
import rospy
from turtlesim.srv import Spawn

def spawn_turtles():
    rospy.wait_for_service('spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        spawn_turtle(5, 5, 0, 'turtle1')
        spawn_turtle(3, 3, 0, 'turtle2')
        spawn_turtle(1, 1, 0, 'turtle3')
        spawn_turtle(7, 7, 0, 'turtle4')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('turtle_spawner')
    spawn_turtles()
