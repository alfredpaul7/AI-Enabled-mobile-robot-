# AI-Enabled-mobile-robot

[AI-Enabled-mobile-robot Repository](https://github.com/alfredpaul7/AI-Enabled-mobile-robot-?utm_source=chatgpt.com)

This project has three major components:

1. Simulation
   The simulation simulates the world in Gazebo. The simulation uses a 3D LiDAR which has been converted to a 2D LiDAR using the Point-LIO package.

2. Real World Robot
   The real world robot uses a Unitree L1 LiDAR, an ODrive motor with inbuilt encoder. The Unitree LiDAR also has a built-in IMU which is fused with the encoder data using Extended Kalman Filter.

3. MoveIt 2 Mobile Manipulation
   A 5-DOF robotic arm has been integrated using MoveIt 2 and ros2_control.
   The robot now supports:

   * Autonomous navigation using Nav2
   * Arm motion planning using MoveIt 2
   * Mobile manipulation
   * RViz interactive planning
   * Gazebo arm simulation
   * Future extensions like pick-and-place and visual servoing

---

# Features

## Navigation Stack

* Nav2 autonomous navigation
* SLAM and localization
* EKF sensor fusion
* Differential drive control
* RViz goal-based navigation

## Manipulation Stack

* MoveIt 2 integration
* 5-DOF robotic arm
* ros2_control support
* JointTrajectoryController
* RViz Motion Planning
* Gazebo simulation support

## Sensors

* Unitree L1 LiDAR
* IMU integration
* Encoder odometry
* Point-LIO 3D to 2D conversion

---

# System Architecture

```text
Nav2 -----------------> Mobile Base Navigation
MoveIt2 --------------> Arm Motion Planning
ros2_control ---------> Joint Control
Gazebo ----------------> Simulation
RViz ------------------> Visualization
EKF -------------------> Sensor Fusion
Point-LIO -------------> LiDAR Processing
```

---

# Requirements

## ROS2 Version

This project runs on ROS2 Jazzy and it is recommended to use the same.

---

# Clone the Repository

```bash
git clone https://github.com/alfredpaul7/AI-Enabled-mobile-robot-.git
```

---

# Source ROS2

```bash
source /opt/ros/jazzy/setup.bash
```

---

# Install Dependencies

## Navigation Dependencies

```bash
sudo apt install \
ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup \
ros-jazzy-robot-localization \
ros-jazzy-gazebo-ros-pkgs
```

## MoveIt 2 Dependencies

```bash
sudo apt install \
ros-jazzy-moveit \
ros-jazzy-moveit-setup-assistant \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-gazebo-ros2-control \
ros-jazzy-joint-trajectory-controller
```

---

# Build and Source the Project

From the project root:

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

---

# Run the Project

# 1. Simulation

To run the Gazebo simulation:

```bash
ros2 launch robot_bringup robot_gazebo_launch.py
```

This launches:

* Gazebo simulation
* Robot State Publisher
* Nav2 stack
* SLAM/localization
* Sensor pipeline

---

# 2. MoveIt 2 Simulation

To launch MoveIt 2:

```bash
ros2 launch mobile_robot_moveit_config move_group.launch.py
```

To launch RViz Motion Planning:

```bash
ros2 launch mobile_robot_moveit_config moveit_rviz.launch.py
```

You can now:

* Plan arm trajectories
* Move the arm interactively
* Execute trajectories
* Visualize planning in RViz

---

# 3. Full Mobile Manipulation System

To launch Nav2 + MoveIt 2 together:

```bash
ros2 launch mobile_robot_bringup full_system.launch.py
```

This launches:

* Gazebo
* Nav2
* MoveIt 2
* ros2_control
* Joint controllers
* RViz

---

# 4. Real World Robot

The real world robot requires a Raspberry Pi SBC.

Clone the repository in the Raspberry Pi and follow the above setup steps.

## On the Main Computer

```bash
ros2 launch robot_bringup robot_computer_launch.py
```

## On the Raspberry Pi

```bash
ros2 launch robot_bringup robot_rpi_launch.py
```

Alternatively, both commands can be run on a single laptop or Raspberry Pi in separate terminals.

---

# Controlling the Robot

The robot can be controlled in three ways.

## 1. Using a Joystick

A PS4 joystick was used for this project.

Connect it via Bluetooth to the computer.

---

## 2. Using Nav2

You can provide a goal pose in RViz and Nav2 will autonomously navigate the robot using:

* LiDAR data
* ODrive odometry
* IMU data

---

## 3. Using MoveIt 2

The arm can be controlled interactively from RViz using the Motion Planning plugin.

Capabilities include:

* Goal pose planning
* Joint-space planning
* Cartesian motion planning
* Collision-aware planning
* Trajectory execution

---

# MoveIt 2 Integration

The mobile manipulator architecture uses:

```text
world
 └── map
      └── odom
           └── base_link
                └── arm_base_link
                     └── link1
                          └── link2
                               └── link3
                                    └── link4
                                         └── link5
                                              └── ee_link
```

The arm is integrated as a separate subsystem while Nav2 continues controlling the mobile base.

This avoids:

* TF conflicts
* Duplicate odometry
* Controller conflicts
* Synchronization issues

---

# Project Structure

```text
AI-Enabled-mobile-robot/
│
├── robot_bringup/
├── robot_description/
├── robot_navigation/
├── robot_localization/
├── mobile_robot_arm_description/
├── mobile_robot_arm_control/
├── mobile_robot_moveit_config/
├── mobile_robot_gazebo/
└── mobile_robot_bringup/
```

---


# Useful Commands

## Build

```bash
colcon build --symlink-install
```

## Source

```bash
source install/setup.bash
```

## View TF Tree

```bash
ros2 run tf2_tools view_frames
```

## View Controllers

```bash
ros2 control list_controllers
```

## View Joint States

```bash
ros2 topic echo /joint_states
```

## Open RViz

```bash
rviz2
```
