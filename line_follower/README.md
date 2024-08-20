# line_follower

## Videos

- [Sample Solution - Sim](https://www.youtube.com/watch?v=tygWRKdvnPo&list=PLWp7_Yk4l1aPcMGxCCvqKCSwnkTBBInI3&index=75)
- [Sample Solution - Real](https://www.youtube.com/watch?v=8AC597dVKHg&list=PLWp7_Yk4l1aPcMGxCCvqKCSwnkTBBInI3&index=80)

## How to Run

### The Skeleton Code (Sim)

1. `export TURTLEBOT3_MODEL=waffle_pi`
2. `roslaunch line_follower line_follower_sim.launch`
3. `rosrun line_follower line_follower_sim.py`

### The Answer (Sim)

1. `export TURTLEBOT3_MODEL=waffle_pi`
2. `roslaunch line_follower line_follower.launch`
3. `rosrun line_follower line_follower_sim_answer.py`

### The Skeleton Code (Real)

1. `bringup` (onboard the robot)
2. `rosrun line_follower my_odom.py`
3. `rosrun line_follower line_follower_real.py`

### The Final Submission (Real)

1. `bringup` (onboard the robot)
2. `rosrun line_follower my_odom.py`
3. `rosrun line_follower line_follower_real_answer.py`

## Instructions

### Sim

1. Using OpenCV, have the simulated robot follow the yellow line in the
   `line_follower` world.

### Real

1. Have the robot follow a black line on the floor.
2. Deal with an obstacle of known dimensions placed on the line.

## PRR Readings

Note: The PRR Readings are especially important for this assignment.

All of Chapter 12 (pp.193-208).

## Notes

1. Make the callback to the image topic subscriber as leightweight as possible.
   In this case, it's not good to follow PRR's line following code, which puts all
   of its logic in the image callback function. This approach works in simulation,
   but when you work with a real robot, you'll quickly run into performance issues.