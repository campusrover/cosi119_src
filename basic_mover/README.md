# basic_mover

## Video

[Sample Solution](https://www.youtube.com/watch?v=VVKwB_DNsKc&list=PLWp7_Yk4l1aPcMGxCCvqKCSwnkTBBInI3&index=72)

## How to Run

### The Skeleton Code

1. `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
2. `rosrun basic_mover my_odom.py`
3. Comment in or out lines under `if __name__=='__main__':` in `basic_mover_answer.py`
   for desired behavior.
4. `rosrun basic_mover basic_mover.py`

### The Final Submission

1. `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
2. `rosrun basic_mover my_odom_answer.py`
3. Comment in or out lines under `if __name__=='__main__':` in `basic_mover_answer.py`
   for desired behavior.
4. `rosrun basic_mover basic_mover_answer.py`

## Instructions

Drive the robot:

1. 1m out, execute a 180-degree turn in-place, and try to get
   the robot back to its original spot.
2. in a square whose sides measure 1m.
3. in a circle with a radius of 1m.

## PRR Readings

Reference:

1. Chapter 2, section "Poses, Positions and Orientations": on frames, positions,
   orientations, and poses pp.26-27;
2. Chapter 6:
   1. section "Actuation: Mobile Platform": on mobile platform actuation
      (pp.77-80);
   2. section "Shaft encoders" on odometry (pp.85-87);
   3. section "Simulators" on simulators in general (pp.92-93);
   4. section "Gazebo" on Gazebo in particular (pp.95-96);
3. Chapter 7, from start till end of section "Creating a Package" on basic
   mobile platform actuation with the Wander-bot (pp.99-103).

## Other Resources

1. [How to determine the distance traveled by a robot by using odometry](https://www.theconstruct.ai/ros-qa-195-how-to-know-if-robot-has-moved-one-meter-using-odometry/).
2. [How to rotate a robot to a desired heading using odometry](https://www.theconstruct.ai/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry)
3. [How to compare two floats in Python using math.isclose()](https://stackoverflow.com/questions/5595425/)

## Notes

1. Press `Ctrl + R` while in Gazebo to reposition the simulated robot to its
   original pose. Spamming the command may sometimes be necessary.

2. Use [format specifiers](https://peps.python.org/pep-0498/#format-specifiers)
   with f-strings to truncate long floats. It's easy to misread `number =
4.38411315886986e-06` as a large value if you miss the exponent `e-06` near the
   end, but `print(f'{number:9.4f}')` yields `0.0000` which is much less
   misleading.
