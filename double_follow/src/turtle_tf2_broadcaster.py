#!/usr/bin/env python
import rospy
import time
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
import turtlesim.srv

def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

# Set the pen color for the turtlesim turtle using turtleX/set_pen service
def set_turtle_color(turtlename, r, g, b):
    rospy.wait_for_service(f'/{turtlename}/set_pen')
    try:
        set_pen = rospy.ServiceProxy(f'/{turtlename}/set_pen', turtlesim.srv.SetPen)
        set_pen(r, g, b, 3, 0)  # (r, g, b, width, off)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle', 'turtle1')

    # Set different colors for each turtle based on its name
    if turtlename == "turtle1":
        set_turtle_color(turtlename, 255, 0, 0)  # Red for turtle1
    elif turtlename == "turtle2":
        set_turtle_color(turtlename, 0, 255, 0)  # Green for turtle2
    elif turtlename == "turtle3":
        set_turtle_color(turtlename, 0, 0, 255)  # Blue for turtle3

    rospy.Subscriber('/%s/pose' % turtlename, turtlesim.msg.Pose, handle_turtle_pose, turtlename)
    rospy.spin()
