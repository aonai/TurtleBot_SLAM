# ME495 Sensing, Navigation and Machine Learning For Robotics
* Sonia Yuxiao Lai
* Winter 2021

# Package List
This repository consists of several ROS packages
- nuturtlesim - A simulator for the robot with Gaussian noise in commanded twist and uniform random noise in wheel slip and for the landmarks with Gaussin noise.
- nuslam - Package to drive the robot with Feature-Based Kalman Filter SLAM, landmark detection from laser scan, and data association. 
- nuturtle_description - Package contianing urdf files and code for the robot.
- rigid2d - rigid2d library that models 2-dimensional differential drive robot.
- trect - Make a turtle draw and follow a rectangular trajectory from user inputs.