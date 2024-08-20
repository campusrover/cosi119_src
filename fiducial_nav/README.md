# fiducial_nav

## Videos

- [Sample Solution - Sim](https://www.youtube.com/watch?v=w7RfK3PKW8M&list=PLWp7_Yk4l1aPcMGxCCvqKCSwnkTBBInI3&index=76)
- [Sample Solution - Real](https://www.youtube.com/watch?v=3K-Q4WguUSE&list=PLWp7_Yk4l1aPcMGxCCvqKCSwnkTBBInI3&index=78)

## How to Run

### The Skeleton Code (Sim)

1. `roslaunch fiducial_nav fiducials_sim.launch`
2. `export TURTLEBOT3_MODEL=waffle_pi`
3. `roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`
4. `rosrun fiducial_nav nav_sim.py`

### The Answer (Sim)

1. `roslaunch fiducial_nav fiducials_sim.launch`
2. `export TURTLEBOT3_MODEL=waffle_pi`
3. `roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch`
4. `rosrun fiducial_nav nav_sim_answer.py`

### The Skeleton Code (Real)

1. `bringup` (onboard the robot)
2. `roslaunch fiducial_nav fiducials_real.launch` (on your vnc)
3. `rosrun rviz rviz` (on your vnc. Tune the RViz to visualize what you want.)
4. `rosrun fiducial_nav nav_real.py`

### The Answer (Real)

1. `bringup` (onboard the robot)
2. `roslaunch fiducial_nav fiducials_real.launch` (on your vnc)
3. `rosrun rviz rviz` (on your vnc. Tune the RViz to visualize what you want.)
4. `rosrun fiducial_nav nav_real_answer.py`

## Instructions (Sim)

1. Study and understand the `mapper.py` file.
2. Complete the `nav_sim.py` file such that the robot:
   1. rotates in place to scan all the target fiducials; and
   2. moves to each target fiducial in turn.

## Instructions (Real)

1. Construct a real environment that matches the simulation world, using real
   fiducials.
2. Change the `fid_ids` parameter of the `mapper.py` file to match the ids of
   your fiducials.
3. Complete the `nav_real.py` file such that the robot:
   1. rotates in place to scan all the target fiducials; and
   2. moves to each target fiducial in turn.

## PRR Readings

1. Chapter 2, sections "tf: Coordinate Transforms" and "Pose, Positions, and Orientations" (pp.25-26)

## Resources

1. [ROS Wiki tutorials on the tf2 package](http://wiki.ros.org/tf2/Tutorials).
2. [This](https://www.rosroboticslearning.com/rigid-body-transformations) may help
   you understand transforms better.
3. The `tf_cheatsheet.md` file found in the lab notebook.

## Notes

1. Pressing CTRL-R to reset your robot's position in Gazebo won't work for this project. This is because CTRL-R causes us to travel back in time in the simulation, which breaks the tf tree and the associated functionalities.

2. Sometimes the tf2 package fails to build the tf tree correctly (e.g., reporting that certain frames that clearly do exist do not, or saying that two frames are not part of the same tf tree when they clearly are). You'll have to simply try relaunching Gazebo, RViz, and your `nav_sim` node in these situations.