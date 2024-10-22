#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    # Create tf buffer and listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Spawn the follower turtle
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)

    leader_turtle = rospy.get_param('~leader_turtle', 'turtle1')  # Param to define the leader
    follower_turtle = rospy.get_param('~follower_turtle', 'turtle2')  # Param to define the follower
    spawner(4, 2, 0, follower_turtle)

    # Publisher for the follower's velocity
    turtle_vel = rospy.Publisher('%s/cmd_vel' % follower_turtle, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Get the transformation between the leader and follower turtles
            trans = tfBuffer.lookup_transform(follower_turtle, leader_turtle, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # Compute the necessary velocity to follow the leader
        msg = geometry_msgs.msg.Twist()
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        # Publish the velocity command
        turtle_vel.publish(msg)

        rate.sleep()
