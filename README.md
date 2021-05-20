# Open-G-SLAM
A general platform for sensor fusion and incorporating different SLAM algorithms.
This project is originally inspired by the [ORB-SLAM]: https://github.com/UZ-SLAMLab/ORB_SLAM3.
ORB-SLAM is a descriptor-based graph-optimization algorithm.
Despite its robust real-time performance on regular hardware, it's not flexible enough.
It is hard to incorporate a new sensor or a different algorithm.

To address this issue, the goal of this project is to build a flexible SLAM system.
Objectives:
  - Redefinition of basic concepts: Pose, Map, Connections
  - Fusing different sensors
  - Integration of different techniques: graph-based SLAM, KLT optical flow, filtering methods, direct/indirect, deep learning, ...
  - Flexible enough to build a cheap tracker with an IMU or a complicated system with Cameras, Range-finders, ...
