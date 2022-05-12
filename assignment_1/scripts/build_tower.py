#! /usr/bin/env python

from cmath import pi
from shutil import move
from turtle import home
from moveit_python import MoveGroupInterface
import rospy
from move_group_interface import MoveGroupPythonInteface
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from tf.transformations import *



###Function to move arm to the give cartesian position
def move_arm(mgpi,target_pose):
    
    waypoints = []
    current_pose = mgpi.get_current_eef_pose().pose

    #setting the current orientation as target orientation
    target_pose.orientation = current_pose.orientation

    #adding target_pose as waypoint
    waypoints.append(target_pose)
 
    (plan,fraction) = mgpi.plan_cartesian_path(waypoints)
    mgpi.display_trajectory(plan)
    mgpi.execute_plan(plan)

###Function to move the arm up to a constant height of 30 cm
def move_up(mgpi):
    waypoints = []
    current_pose = mgpi.get_current_eef_pose().pose
    current_pose.position.z = 0.3
    waypoints.append(current_pose)
    (plan,fraction) = mgpi.plan_cartesian_path(waypoints)
    mgpi.display_trajectory(plan)
    mgpi.execute_plan(plan)

###Function to move the arm in x-y direction to target at a set height
def move_static(mgpi,target_pose,tower_height):
    waypoints = []
    target_pose.position.z = tower_height
    target_pose.orientation = mgpi.get_current_eef_pose().pose.orientation
    waypoints.append(target_pose)
    (plan,fraction) = mgpi.plan_cartesian_path(waypoints)
    mgpi.display_trajectory(plan)
    mgpi.execute_plan(plan)


##Defining object class for the blocks to store its pose and id
class Block:
    def __init__(self,pose,id) :
        self.pose = pose
        self.id   = id


   

def main():
    
    try:
        mgpi = MoveGroupPythonInteface()          #initializing the mgpi interface
        def callback(input_msg):
            input("Enter to Initiate")

            #getting goal and block positions
            goal = []
            block = []
            for x in input_msg.pose[2:4]:
                goal.append(x)
            for x,y in zip(input_msg.pose[4:10],input_msg.name[4:10]):
                block.append(Block(x,y))

            #sorting the list to pick the block at the farthest y-axis            
            goal.sort(key=lambda x: x.position.y, reverse=True)
            block.sort(key=lambda x: x.pose.position.y, reverse=True)
            
            #setting an optimal home position
            home_state = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]

            block_count = 1
            current_goal = goal[0]
            tower_height = 0.09
            block_height = 0.082
            mgpi.move_to_joint_state(home_state)    #moving to home position
           
            input("Start Building")
            #iterating throught the blocks
            for current_block in block:

                current_block.pose.position.z = block_height
                move_up(mgpi)
                mgpi.move_to_joint_state(home_state)

                #First Tower
                if block_count<=3:
                    print ("moving",current_block.id)
                    move_arm(mgpi,current_block.pose)
                    mgpi.close_gripper(current_block.id)
                    move_up(mgpi)
                    move_static(mgpi,current_goal,tower_height)
                    current_goal.position.z = tower_height
                    move_arm(mgpi,current_goal)
                    mgpi.open_gripper(current_block.id)

                #Second Tower
                elif block_count <=6:
                    print ("moving",current_block.id)
                    current_goal = goal[1]                          #change to second goal position
                    move_arm(mgpi,current_block.pose)
                    mgpi.close_gripper(current_block.id)
                    move_up(mgpi)
                    move_static(mgpi,current_goal,0.39)             #arm height is the tower height of 1st tower
                    current_goal.position.z = tower_height
                    move_arm(mgpi,current_goal)
                    mgpi.open_gripper(current_block.id)

                elif block_count >6:
                    print("Tower Built!")
                    quit()

                block_count+=1
                tower_height+=0.09
                if block_count == 4:
                    tower_height = 0.09
                move_up(mgpi)
                
        #Subscribing to the model states
        rospy.Subscriber("gazebo/model_states",ModelStates,callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
