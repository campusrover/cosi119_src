#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

'''
A callback function that subscribes to the given bot's odometry information
and converts it to a transform object with the parent as world (0,0).
'''
def handle_bot_pose(msg, botname):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = botname
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)


'''
Takes in 4 parameters corresponding to the names of the robots, and publishes their transform
objects as child nodes to the same 'world' parent.
'''
if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    botnames = [rospy.get_param('~bot1'), rospy.get_param('~bot2'), rospy.get_param('~bot3'), rospy.get_param('~bot4')]
    for botname in botnames:
        rospy.Subscriber('/'+botname+'/odom', Odometry, handle_bot_pose, botname)
    rospy.spin()