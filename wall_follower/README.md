# wall_follower

## Video

[Sample Solution](https://www.youtube.com/watch?v=yH474O4mAdw&list=PLWp7_Yk4l1aPcMGxCCvqKCSwnkTBBInI3&index=76)


### The Skeleton Code

1. `roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch`
2. Move robot to the south of the southern wall in stage_1.
3. `rosrun wall_follower wall_follower.py`

### The Final Submission

1. `roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch`
2. Move robot to the south of the southern wall in stage_1.
3. `rosrun wall_follower wall_follower_answer.py`

## Instructions

1. Subsribe to the `scan` topic to receive `LaserScan` messages.
2. Move the robot to the south of the souther wall in the `turtlebot3_stage_1` world.
3. Circumnavigate the wall by maintaining a constant distance.
4. Use PID control.
5. Demonstrate this in SIM.

## PRR Readings

Reference:

1. Chapter 6:
   1. section "Sensors", till start of "Visual Camera" subsection (pp.81-82).
   2. section "Laser Scanners" (p. 85).
2. Chapter 7, from section "Reading Sensor Data" to the end (pp. 103-108)

## Other Resources

1. [Robotics Programming: PID Algorithm](https://www.youtube.com/watch?v=dynSWBXu9aA): Shows how to implement the math of PID control in code, and gives a high-level overview of the algorithm.
2. [How to move objects in Gazebo using its GUI](https://answers.gazebosim.org//question/13445/how-to-move-objects-and-robot-models-using-gui/)
3. [Difference between Gazebo and RViz](https://answers.ros.org/question/200044/different-between-gazebo-and-rviz/)

## Notes

1. Get a feel for what kind of data the `scan` topic publishes by moving the robot around in Gazebo and executing `rostopic echo scan` on a separate terminal.

2. Change the `fixed_frame` in RViz from `odom` to `base_link` to make RViz graphically display the LIDAR readings.
