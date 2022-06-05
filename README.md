# MTRX5700 Major - README
![alt text](https://assets.entrepreneur.com/content/3x2/2000/1639070742-GettyImages-1157937604.jpg)

## Instructions

This git repo contains all the code necessary for Group 11's MTRX5700 Major Project, The Automated Kasparov Chess Playing Robot.
Unfortunately we were unable to control the robot arm from Ubuntu 20.04 and ROS Noetic, however our vision processing workflow did not work on Ubuntu 18.04 and ROS Melodic. Hence to successfully utilise this project, one needs two computers connected to the same local area network which the UR5e Arm is connected to. The first computer(henceforth called the Arm computer) will need to run ROS Melodic and Ubuntu 18.04. The Arm computer will be responsible for actually communicating with UR5e arm. The second computer(henceforth called the User computer) will need to run ROS Noetic and Ubuntu 20.04. The user interface be responsible for processing image data, running the user interface and communicating general control commands to the Arm Computer. One of the computers also needs to be connected to an IntelRealSense, which we used to get the images for chess detection. This can be connected to ROS using the realsense-ros package available at https://github.com/IntelRealSense/realsense-ros. It may well be possible to use other camera setups, but this is not officially supported.

## Instructions - User Computer

The user computer code is included in the detect-decide branch of this Git Repo. This should be 
