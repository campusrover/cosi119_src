
# Turtle Following Simulation using ROS and TF2

## Project Overview

This project simulates a fun scenario where multiple turtles follow a designated leader turtle using the Robot Operating System (ROS) and the `tf2` package for managing coordinate transformations. The main goal is to demonstrate how coordinate systems work in a robotic context and how turtles can dynamically respond to each other's positions in real time.

### Key Features
- **Leader-Follower Behavior**: One turtle is controlled by the user, while others automatically follow it based on their position.
- **Dynamic Transformations**: Utilizes the `tf2` package to manage and calculate the positions and orientations of each turtle in the coordinate system.
- **Extendable Design**: Easily add more turtles to create chains of followers.

## Learning Objectives
- Understand the need and role of coordinate systems in robotics.
- Comprehend coordinate system transformations.
- Gain practical experience using the `tf2` package in ROS.

## System Requirements
- ROS Noetic or Melodic
- Python 3
- `turtlesim` package
- `tf2` package

## Installation

### Prerequisites
Ensure you have ROS installed and set up properly on your machine. You can follow the official ROS installation guide for your operating system.


### Build the Package
Navigate to your ROS workspace and build the package:
```bash
cd ~/catkin_ws/src
catkin_make
```

### Source the Workspace
```bash
source ~/catkin_ws/devel/setup.bash
```

## Package Structure
```plaintext
double_follow/
├── launch/
│   └── tf2_demo.launch
├── src/
│   ├── turtle_tf2_broadcaster.py
│   └── turtle_tf2_listener.py
├── README.md
|── video.txt
├── CMakeLists.txt
└── package.xml
```

- **launch/**: Contains the launch file to start the turtlesim and the broadcasting/listening nodes.
- **src/**: Contains Python scripts for broadcasting turtle positions and listening to those transforms.

## Usage

### Launch the Simulation
To start the simulation, run the following command:
```bash
roslaunch double_follow tf2_demo.launch
```

### Controls
- Use the keyboard to control the leading turtle (`turtle1`).
- The other turtles will automatically follow the leader based on the transformations received.

### Observing the Behavior
You should see `turtle1` moving according to your keyboard inputs while `turtle2` and `turtle3` follow it, maintaining a formation as they chase after the leader.

## Code Explanation

### 1. Launch File (`tf2_demo.launch`)
This file launches all required nodes. It includes:
- The `turtlesim_node`, which creates the turtles.
- A teleoperation node (`turtle_teleop_key`) for user control of `turtle1`.
- Three instances of `turtle_tf2_broadcaster.py`, each broadcasting the position of `turtle1`, `turtle2` and `turtle3`.
- A listener node (`turtle_tf2_listener.py`) that controls the following turtles.

### 2. Broadcaster (`turtle_tf2_broadcaster.py`)
- This script handles the broadcasting of each turtle's position.
- It listens to the pose of the turtle, converts it into a transform, and publishes the position and orientation using quaternions.

### 3. Listener (`turtle_tf2_listener.py`)
- This script listens for the transforms between the leader and the follower turtles.
- It calculates the necessary velocities to make the follower turtles chase the leader smoothly, utilizing basic trigonometry to steer them.

## Video Demonstration
A demonstration of the simulation can be found at the following link:
- [Video URL](https://drive.google.com/file/d/11_Ma95UvN1ZkKoQCxhAP2zgx1LLK9loB/view?usp=sharing)

