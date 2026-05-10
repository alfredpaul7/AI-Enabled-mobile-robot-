# AI-Enabled-mobile-robot

This project has two components

1. Simulation:
    The simulation simulates the world in gazebo. The simulation uses a 3D lidar which has been converted to a 2D lidar using the Point-lio package.

2. Real World Robot:
    The real world robot uses an Unitree-l1 lidar, a Odrive motor with inbuilt encoder. The Unitree-li lidar also has a built in IMU which is fused with the encoder data using Extended Kalman Filter.

How to run it.

## Clone the repository.

> git clone https://github.com/alfredpaul7/AI-Enabled-mobile-robot-.git

## Source ros2 

This project runs on ros2 Jazzy and it is recommended to use the same. Run

> source /opt/ros/Jassy/setup.bash

## Build and source the project

From the project root, run

> colocn build --symlink-install
> source install/setup.bash

## Run the project

### Simulation.

To run the simulation, run 

> ros2 launch robot_bringup robot_gazebo_launch.py

### Real world robot.
The real world robot requires an raspberry pi sbc. Clone the repository in a raspberry pi and follow the above steps.

In the computer run

> ros2 launch robot_bringup robot_computer_launch.py

In the raspberry pi, run

> ros2 launch robot_bringup robot_rpi_launch.py

Alternatively, you can run both the commands on a single laptop or raspberry pi itself in new terminals.

## Controlling the robot.

The robot can be controlled in two ways,

1. Using a Joystick:
    A PS4 Joystick was used for this project. Connect it via bluetooth to any the computer
2. Using Nav2
    You can give a goal pose in the rviz2 window and Nav2 will autonomously run the robot using the data from the lidar, odrive odom and IMU data.

