#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
import numpy as np

class MazeSolverMapping:
    def __init__(self):
        rospy.init_node('maze_solver_with_mapping')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        # Map for tracking visited places (Occupancy Grid)
        self.grid_size = 100
        self.map_resolution = 0.1  # Each cell represents 10 cm
        self.visited_map = np.zeros((self.grid_size, self.grid_size))  # Initialize grid map
        
        self.position = [0, 0]  # Initialize robot position
        self.prev_error = 0.0

        # PID controller parameters
        self.kp = 1.0
        self.kd = 0.5
        self.desired_distance = 0.5
        self.rate = rospy.Rate(10)

    def scan_cb(self, msg):
        # Get front and right distances from LIDAR
        front_distance = min(min(msg.ranges[0:30]), min(msg.ranges[-30:]))
        right_distance = min(msg.ranges[90:120])
        
        # Process wall following and obstacle avoidance
        self.follow_wall(front_distance, right_distance)

    def odom_cb(self, msg):
        # Get robot's current position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Update position and map grid based on resolution
        grid_x = int((x + self.grid_size * self.map_resolution / 2) / self.map_resolution)
        grid_y = int((y + self.grid_size * self.map_resolution / 2) / self.map_resolution)
        
        if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
            # Mark current position as visited
            self.visited_map[grid_x, grid_y] = 1

        self.position = [grid_x, grid_y]

    def follow_wall(self, front_distance, right_distance):
        twist = Twist()

        # Right-Hand Rule Wall Following
        if right_distance == math.inf or right_distance == 0:  # No wall on the right
            twist.linear.x = 0.2
            twist.angular.z = -0.5  # Turn right to find the wall
        else:
            # PID control for keeping distance to wall
            error = right_distance - self.desired_distance
            derivative = error - self.prev_error
            angular_v = self.kp * error + self.kd * derivative

            if front_distance < 0.8:  # Obstacle ahead
                twist.linear.x = 0.1  # Slow down
                twist.angular.z = 0.8  # Turn left to avoid obstacle
            else:
                twist.linear.x = 0.2  # Move forward
                twist.angular.z = angular_v

            self.prev_error = error
        
        # Publish movement
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()

    def save_map(self):
        # Save visited map as an image or data file (optional)
        np.savetxt('visited_map.txt', self.visited_map, fmt='%d')

if __name__ == '__main__':
    try:
        maze_solver = MazeSolverMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
