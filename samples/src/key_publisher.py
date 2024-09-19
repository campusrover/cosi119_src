#!/usr/bin/env python

# termios library allows access to terminal (shell) input 
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

# Convention that means: "if this program was run directly (vs. imported)"
if __name__ == '__main__':
    key_pub = rospy.Publisher('keys', String, queue_size=1) 
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)

    # Save "keyboard attributes" so you can restore them at the end.
    old_attr = termios.tcgetattr(sys.stdin) 
    tty.setcbreak(sys.stdin.fileno())
    print(">")

    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]: 
                ch = sys.stdin.read(1)
                key_pub.publish(ch)
            rate.sleep()
    except KeyboardInterrupt:
        print("\nCtrl+C pressed...")

    # restore keyboard attributes
    print("exiting!")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)