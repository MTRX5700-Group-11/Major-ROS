# MTRX5700 Assignment 1 - README
![alt text](https://assets.entrepreneur.com/content/3x2/2000/1639070742-GettyImages-1157937604.jpg)

## Instructions

This folder contains skeleton code to build the tower for Question 2.  

- **You need to write your code in the file assignment_1/scripts/build_tower.py**.
- The script assignment_1/scripts/demo.py demonstrates the control of the UR5e arm over ROS. Please use this as a reference.

Read carefully the description for what you need to achieve, test early and often, ask your tutor lots of questions, and have fun! :)

### Setting up the framework
This code was tested on Ubuntu 18.04 with ROS Melodic and Gazebo 9.0.0.
Please try to match this configuration if you are using your personal computers.
The code may also work on ROS Kinetic, but it has not been tested. The code may also be upgraded to ROS Noetic and Python3, but it also has not been tested.

The following instructions assume that you have a basic understanding of ROS.
If you are new to ROS, please read the article *[An Absoulte Beginner's Introduction to ROS](https://github.com/tejaswid/guides/tree/master/intro_to_ros)*.
For a slightly more detailed introduction you can follow [this](https://rsl.ethz.ch/education-students/lectures/ros.html) lecture series.

#### Pre-requisites (Only for personal computers. If you are using computers in the MXLab or the VirtualBox image, skip this step)
*Note*: These dependencies will also help in future assignments.

1. Install [Ubuntu 18.04](https://ubuntu.com/download/desktop).
2. Install ROS Melodic following the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
3. Install dependencies
```bash
sudo apt-get install socat libeigen3-dev libyaml-cpp-dev libboost-dev python-lxml libsoqt4-dev libcoin80-dev libqt4-dev libblas-dev liblapack-dev libqhull-dev python-pip python-catkin-tools python-pymodbus
```
```bash
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-soem ros-melodic-socketcan-interface ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools ros-melodic-moveit-python ros-melodic-moveit-sim-controller ros-melodic-moveit-resources ros-melodic-actionlib ros-melodic-derived-object-msgs ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-eigen-conversions ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-control-msgs ros-melodic-controller-interface ros-melodic-controller-manager ros-melodic-dynamic-reconfigure ros-melodic-effort-controllers ros-melodic-force-torque-sensor-controller ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-geometry-msgs ros-melodic-hardware-interface ros-melodic-joint-state-controller ros-melodic-joint-state-publisher ros-melodic-joint-trajectory-controller ros-melodic-message-generation ros-melodic-message-runtime ros-melodic-moveit-core ros-melodic-moveit-fake-controller-manager ros-melodic-moveit-kinematics ros-melodic-moveit-planners-ompl ros-melodic-moveit-ros-manipulation ros-melodic-moveit-ros-move-group ros-melodic-moveit-ros-planning ros-melodic-moveit-ros-visualization ros-melodic-moveit-simple-controller-manager ros-melodic-pluginlib ros-melodic-realtime-tools ros-melodic-robot-state-publisher ros-melodic-roscpp ros-melodic-sensor-msgs ros-melodic-std-srvs ros-melodic-tf ros-melodic-tf-conversions ros-melodic-tf2-geometry-msgs ros-melodic-tf2-msgs ros-melodic-tf2-ros ros-melodic-trajectory-msgs ros-melodic-urdf ros-melodic-velocity-controllers ros-melodic-xacro
```
```bash
pip install argparse rosdep matplotlib mpmath numpy scikit-learn scipy
pip install torch==1.5.0+cpu torchvision==0.5.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
```

4. Initialize and update rosdep
```bash
sudo rosdep init
rosdep update
```

#### Setting up catkin workspace
The computers in MXLab have the above configuration. All the depedencies for this code to run have been installed. Please type the following commands in a terminal one after the other.
1. Source ROS commands
```bash
source /opt/ros/melodic/setup.bash
```
2. Create and initialize new catkin workspace. You may choose any name you like.
Here we chose **`ur5espace`** and it is located in the home directory.  
```bash
mkdir -p ur5espace/src && cd ur5espace
catkin init
```
3. Download this folder, unrip and copy all the subfolders to `ur5espace/src`
```bash
cp -r <location_of _downloaded _folder>/* src/
```
4. Install dependencies using rosdep. Run the following from the folder `ur5espace`
```bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```
6. Build the packages in the catkin workspace  
```bash
catkin build
```
7. Source the workspace
```bash
source devel/setup.bash
```
8. Grant executable permissions to the necessary scripts  
```bash
chmod +x src/assignment_1/scripts/demo.py src/assignment_1/scripts/build_tower.py
```
9. If everything builds correctly, you are good to go. Else, first search for the issue on the internet and if you still cannot find a solution, post it on EdStem.

### Running the demo
#### In simulation
Open 4 terminals and source ROS commands in all of them.
```bash
source ur5espace/devel/setup.bash
```
1. Terminal 1 - Launch the simulated environment in gazebo
```bash
roslaunch assignment_1 ur5e_ass1.launch
```
2. Terminal 2 - Launch the MoveIt! Planner and Controller **with sim:=true**. Wait until you see a green text saying *You can start planning now!*
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true
```
3. Terminal 3 - Launch RViz to visualize the planning scene **with config:=true**
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```
4. Terminal 4 - Launch the demo (or your code when ready)
```bash
rosrun assignment_1 demo.py
```
or
```bash
rosrun assignment_1 build_tower.py
```

**Explanation**:  
1. `roslaunch assignment_1 ur5e_ass1.launch` opens Gazebo (simulation software) and loads the simulated world with the robot, blocks and obstacles.  
2. `roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true` launches Moveit!, loads essential kinematics and dynamics of the robot arm and initializes the *planner*, to plan trajectories that the arm should move in (avoiding obstacles), and the *controller*, to specify how each joint should move to traverse a desired trajectory.  
3. `roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true` is solely to visualize the environemnt from the Robot's point of view, i.e. what it's current state is, where it thinks objects are in the environment, and what its planned trajectory will be. You should check that the current state of the robot matches that of the one in Gazebo.
4. `rosrun assignment_1 demo.py` runs the actual script that tells the robot arm what to do.

#### On Hardware
Open 5 terminals and source ROS commands in all of them.
```bash
source ur5espace/devel/setup.bash
```
1. Terminal 1 - roscore
```bash
roscore
```
2. Terminal 2 - Launch the robot driver with the correct IP, kinematic configuration and **limited:=true**. Check with your tutor for the correct IP.
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.10 limited:=true kinematics_config:=~/ur5espace/src/universal_robot/ur_e_description/config/ur5e_calib.yaml
```
3. Terminal 3 - Launch the MoveIt! Planner and Controller **with sim:=false**. Wait until you see a green text saying *You can start planning now!*
```bash
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=false
```
4. Terminal 4 - Launch RViz to visualize the planning scene **with config:=true**
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```
5. Terminal 5 - Launch the demo (or your code when ready)
```bash
rosrun assignment_1 demo.py
```
or
```bash
rosrun assignment_1 build_tower.py
```

**Explanation**:  
1. The key changes here are that we launch the robot driver instead of the gazebo simulation and as a result we set the **sim** flag to *false* when launching the MoveIt! planner and controller.

**Pro Tips**:  
1. Use the software Terminator to neatly stack terminals.  
2. Use an IDE (pycharm, visualcode, eclipse, qtcreator) to get code completion.  
3. DO NOT use Anaconda w/ ROS - it does not behave well and causes some weird glitches in the matrix.


## Guidelines for writing your code
1. Please write all your code in the file assignment_1/scripts/build_tower.py  
2. You may use any number of custom defined functions as you like.  
3. Necessary modules have already been imported. You are allowed to import only system or standard python modules. If you need any special modules, please check with your tutors.  
4. Please write clean code and comment it sufficiently. If the tutors cannot understand your code, they will not be able to grade your work. 
5. You may be tempted to use the pick-and-place tutorial from the online MoveIt! tutorials, but be warned that that will not work.
6. Subscribe to ROS topic "/gazebo/model_states" to get the blocks and goal positions.

## Information about the gripper in Simulation
1. **Gripper Animation:** The gripper in the simulation is not animated. You will not see the gripper open and close when you use the close_gripper and open_gripper functions. However, the functions work and the blocks will be picked up.

2. **Gripper Functions:** The gripper functions, close_gripper and open_gripper from the move_group_interface class should be used with an argument as `close_gripper("object_name")` and `open_gripper("object_name")`.  The object whose name is object_name will then be picked up or released respectively. If no argument is provided, then the gripper will not pick or release any object. The objects in the assignment are named block_1, block_2, ..., block_5. The following figure shows the layout of the blocks.

3. **Picking up blocks:** There is an issue with the gripper colliding with the blocks if they are picked up from a height less than 0.08m (height of the block). Therefore please pick the blocks up from a height slightly above the blocks (e.g. 0.086m). You will see that when the blocks are picked up, they will float in the air close to the gripper and move along with it.

4. **Placing the blocks:** The motion planner (in MoveIt!) will fail if the blocks collide while placing them. To avoid this, please plan your paths with suitable waypoints to avoid collisions. You may also release the blocks from a slightly higher position than necessary (see next two points for limitations).

5. **RViz and Gazebo:** While Gazebo looks nice and fancy, please don't ignore RViz. The visualization that you see in RViz (green objects) shows what the robot think's the current scene looks like and what its planned motion is. This is used by Moveit! to plan the robot's path and avoid obstacles. As we do not have any external sensor (e.g. camera) to tell the robot where the blocks are located, the initial state of these blocks is hard coded. Hence, in the assignment they are updated when the gripper functions are called, by simulating a fake gripper (gazebo_attach.py).

6. **No gravity in RViz:** There is no gravity in RViz. So if you pickup a block and release it from a height, you will see the block fall down in Gazebo but in RViz the block is still floating in the air. So, even though the block actually fell down, the robot does not know this and thinks that the block is still in the air. This will effect the way the planner works. If you are building the tower by dropping the blocks from a height to prevent the collision problem, please keep this height offset small, and take note of it when you plan your waypoints.

7. **Demo on the real robot:** We will try to tweak the real robot to overcome the above limitations in the simulation so that the code you submit works directly on the real robot.


## Code Overview

This folder has 5 ROS packages  
1. **assignment_1** - Core files for assignment 1  
2. **Universal_Robots_ROS_Driver** - Driver to interact with the robot arm over ROS
3. **Universal_Robots_Client_Library** - C++ library for Universal Robots interfaces  
4. **universal_robot** - Robot description files and utilities for simulation  
5. **robotiq** - Driver and controller to interact with the gripper over ROS  
6. **gazebo_ros_link_attacher** - Untilities to interact with objects in Gazebo  

For the purpose of this assignment, you only need to look at the folder **assignment_1/scripts**.  
This folder has the following files.  
1. **build_tower.py** - The code that you need to complete to build the tower.  
2. **demo.py** - Demonstrates how to control the arm. Use this as a reference.  
3. **move_group_interface.py** - Class that forms the interface between your code and the MoveIt! package.  
4. **create_stage** - Class used to build the PlanningScene. **Take a look at how the block was added and try to add other objects for the bonus points**.  
5. **gripper_control** - Class that exposes control of the Robotiq gripper.  
6. **gazebo_attach** - Class that enables a hack to attach objects in Gazebo, to simulate pick and place without physics.  

The simulated environment for Gazebo is described in **assignment_1/world/stage.sdf** and the file **assignment_1/launch/ur5e_ass1.launch** launches this world.

If you are interested in learning how this assignment was built, or interested in extending it, you can refer to [this](https://github.com/tejaswid/guides/tree/master/intro_to_ur5e_gazebo) guide.
