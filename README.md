# MTRX5700 Major - README
![alt text](https://assets.entrepreneur.com/content/3x2/2000/1639070742-GettyImages-1157937604.jpg)

## Instructions

This git repo contains all the code necessary for Group 11's MTRX5700 Major Project, The Automated Kasparov Chess Playing Robot.
Unfortunately we were unable to control the robot arm from Ubuntu 20.04 and ROS Noetic, however our vision processing workflow did not work on Ubuntu 18.04 and ROS Melodic. Hence to successfully utilise this project, one needs two computers connected to the same local area network which the UR5e Arm is connected to. The first computer(henceforth called the Arm computer) will need to run ROS Melodic and Ubuntu 18.04. The Arm computer will be responsible for actually communicating with UR5e arm. The second computer(henceforth called the User computer) will need to run ROS Noetic and Ubuntu 20.04. The user interface be responsible for processing image data, running the user interface and communicating general control commands to the Arm Computer. One of the computers also needs to be connected to an IntelRealSense, which we used to get the images for chess detection. This can be connected to ROS using the realsense-ros package available at https://github.com/IntelRealSense/realsense-ros. It may well be possible to use other camera setups, but this is not officially supported.

## Instructions - User Computer

The user computer code is included in the detect-decide branch of this Git Repo. This should be 




### Instructions to control the arm - Arm Computer
The following instructions are for setting up the arm interface on an ubuntu 18.04 computer, so that the user computer can publish topics to this computer, which will be published to the ur5e arm.

### Initializing the Hardware

1. Make sure the UR5e arm is turned on and connected to the network, note down the ip address of the arm, `robot ip` from the settings.
2. Load the program `mtrx5700_gripper`
3. Connect the gripper to the computer using an USB cable
4. Check if the computer is connected to the same network
5. Find the ip address of the computer through `ssh` and add it to `.bashrc` with the right syntax.
6. Make any one of the computers as the master and run `roscore` on the master computer. Only one computer should be the master.
7. Save the file `mxlab_calib.yaml` into your `src` folder

#### In simulation
Open 7 terminals and source ROS commands in all of them.
1. Terminal 1 Launching the Arm drivers
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:= "robot ip" limited:=true kinematics_config:=/home/your_ws/src/mxlab_calib.yaml
```
`robot ip` should be the ip of the arm

2. Terminal 2 - Launch the Planner

Wait for the previous terminal to load successfully
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=false
```
3. Terminal 3 - Launch RViz to visualize the planning scene **with config:=true**
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```
4. Configure the usb port
```bash
sudo chmod 777 /dev/ttyUSB0
```
4. Terminal 4 - Launch the Gripper node, make sure the light is turned blue
```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
5. Terminal 5- Run the arm program
```bash
rosrun assignment_1 chess_arm.py
```
Now you are all set! Wait for the user computer to publish topics to the arm and see its action in playing chess.

Note: It is very important to double-check that all the devices are connected to the same network
