# Open-G-SLAM
A general platform for sensor fusion and incorporating different SLAM algorithms.
This project is originally inspired by the [ORB-SLAM](https://github.com/UZ-SLAMLab/ORB_SLAM3).
ORB-SLAM is a descriptor-based graph-optimization algorithm.
Despite its robust real-time performance on regular hardware, it's not flexible enough.
It is hard to incorporate a new sensor or a different algorithm.

To address this issue, the goal of this project is to build a flexible SLAM system.
Objectives:
  - Redefinition of basic concepts: Pose, Map, Connections
  - Fusing different sensors
  - Integration of different techniques: graph-based SLAM, KLT optical flow, filtering methods, direct/indirect, deep learning, ...
  - Flexible enough to build a cheap tracker with an IMU, or a complicated system with Cameras, Range-finders, ...

# Dependencies

## Core Library
### 1. C++14
### 2. OpenCV (4.2.0)
### 3. Eigen3+
### 4. Ceres Solver
### 5. g2o
### 6. Boost
Filesystem, serialization, etc.
### 7. Google Logging library (glog)

## ROS
### 1. Tested with ROS Noetic under Ubuntu 20.04 LTS
Better to use a ROS distro with compatible OpenCV (with Core Library).
### 2. DVS Event Messages
For event-based SLAM, install DVS Event Messages according to the instructions.

# Installation
To make core library independent of ROS echo-system, this library is built as
a separate package. If ROS extension desired, you should first build and install
core library and then link it with ROS package.

## 1. Build and Install Core Library

## 2. Link ROS Library
This can be done in 2 ways:
### - Using catkin_make
Copy the content of ros folder in catkin work space directory as a ROS package.
Then issue a catkin_make command.
### - Build with Core Library
It is possible to build and link ROS package with core library.
In project root CMakeLists.txt file, comment out ROS build.
Then build and install core library so that find_package can recognize it.
Finally, enable ROS build and build the project one more time.