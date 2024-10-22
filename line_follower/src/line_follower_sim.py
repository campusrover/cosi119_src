#!/usr/bin/env python3
import rospy
import cv2 
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class LineFollowerSim:
    def __init__(self):
        cv2.namedWindow("window", 1)  # Create a window to display images
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.current_position = None  # Store current position
        self.centroid_x = None  # Initialize centroid_x
        self.state = 'following_line'  # Initialize state

    def image_callback(self, msg):
        """Callback to `self.image_sub`."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Create a mask for the yellow line (adjust HSV range as needed)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        rospy.loginfo("IMAGE CALLBACK CALLED!")
        self.process_line(mask)

    def process_line(self, mask):
        """Process the mask image to find the centroid of the yellow line."""
        # Calculate the moments of the mask
        M = cv2.moments(mask)
        if M['m00'] != 0:
            self.centroid_x = int(M['m10'] / M['m00'])  # x coordinate of centroid
            rospy.loginfo(f"Found centroid at x={self.centroid_x}")
        else:
            self.centroid_x = None  # No valid centroid found
            rospy.logwarn("No valid centroid found.")

    def scan_callback(self, msg):
        """Callback for LaserScan data."""
        ranges = msg.ranges
        if min(ranges) < 0.5:  # Example threshold for obstacle detection
            self.state = 'avoiding_obstacle'
            rospy.loginfo("Obstacle detected! Switching to avoiding_obstacle state.")
        else:
            self.state = 'following_line'
            rospy.loginfo("No obstacle detected. Continuing to follow the line.")

    def follow_line(self):
        """Follow the line based on the centroid position."""
        if self.state == 'following_line' and self.centroid_x is not None:
            image_width = 640  # Replace with the actual width of your image
            error = self.centroid_x - image_width / 2  # Calculate the error from the center
            speed = Twist()
            speed.linear.x = 0.2  # Set a constant forward speed

            # Proportional control for steering
            kp = 0.005  # Proportional control gain
            speed.angular.z = -error * kp
            speed.angular.z = max(min(speed.angular.z, 1.0), -1.0)  # Clamp angular speed

            self.cmd_vel_pub.publish(speed)
            rospy.loginfo(f"Following line: linear.x={speed.linear.x}, angular.z={speed.angular.z}")
        elif self.state == 'avoiding_obstacle':
            self.avoid_obstacle()
        else:
            self.stop_robot()

    def stop_robot(self):
        """Stop the robot when no line is detected."""
        speed = Twist()
        speed.linear.x = 0.0  # Stop moving forward
        speed.angular.z = 0.0  # Stop turning
        self.cmd_vel_pub.publish(speed)
        rospy.loginfo("Robot stopped.")

    def avoid_obstacle(self):
        """Avoid an obstacle."""
        speed = Twist()
        speed.linear.x = 0.0  # Stop moving forward
        speed.angular.z = 0.5  # Turn to avoid the obstacle
        self.cmd_vel_pub.publish(speed)
        rospy.loginfo("Avoiding obstacle.")

    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.follow_line()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('line_follower_sim')
    LineFollowerSim().run()
