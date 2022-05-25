#! /usr/bin/env python

from cmath import pi
from email.policy import default
from pickle import NONE
import time
from re import S
from shutil import move
from turtle import home
from moveit_python import MoveGroupInterface
import rospy
sim = True
if sim:
    from move_group_interface_chess import MoveGroupPythonInteface #this is for simulation
else:
    from move_group_interface import MoveGroupPythonInteface #this is for reality
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from tf.transformations import *

class chess_arm():
    def __init__(self):
        self.mgpi = MoveGroupPythonInteface() 
        #Optimal home position
        self.home_state = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
        #setting the movement height to be 7cm to avoid any collision
        self.cube_height = 0.018
        #x,y position of A8 --> x = 0.21 ;y = 0.44 
        self.a8_position = [0.21,0.44]
        #square width in m
        self.square_width = 0.06
        #position to put the removed piece
        self.remove_position = 'J4'


    #function to move the arm to home position
    def move2home(self):
        self.mgpi.move_to_joint_state(self.home_state)

    ### Function to convert chess square position (A1 to H8) to x,y
    def square2xy(self,square):

        square_id = list(square.upper())

        x_position = self.a8_position[0]-((ord(square_id[0])-ord('A'))*self.square_width)

        y_position = self.a8_position[1]+((8-int(square_id[1]))*self.square_width)

        return x_position,y_position

    ###Function to move arm to the give cartesian position
    def move_arm(self,target_pose):
        
        waypoints = []
        current_pose = self.mgpi.get_current_eef_pose().pose

        #setting the current orientation as target orientation
        target_pose.orientation = current_pose.orientation

        #adding target_pose as waypoint
        waypoints.append(target_pose)
    
        (plan,fraction) = self.mgpi.plan_cartesian_path(waypoints)
        self.mgpi.display_trajectory(plan)
        self.mgpi.execute_plan(plan)
    
    def make_pose(self,xy):
        print("x: {} Y: {}".format(xy[0],xy[1]))
        pose = Pose()
        pose.position.x = xy[0]
        pose.position.y = xy[1]
        pose.position.z = self.cube_height
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        return pose



    ###Function to move the arm up to a constant height of 30 cm
    def move_up(self):
        waypoints = []
        current_pose = self.mgpi.get_current_eef_pose().pose
        current_pose.position.z = 0.2
        waypoints.append(current_pose)
        (plan,fraction) = self.mgpi.plan_cartesian_path(waypoints)
        self.mgpi.display_trajectory(plan)
        self.mgpi.execute_plan(plan)

    ###Function to move the arm in x-y direction to target at a set height
    def move_static(self,target_pose):
        waypoints = []
        target_pose.position.z = self.cube_height
        target_pose.orientation = self.mgpi.get_current_eef_pose().pose.orientation
        waypoints.append(target_pose)
        (plan,fraction) = self.mgpi.plan_cartesian_path(waypoints)
        self.mgpi.display_trajectory(plan)
        self.mgpi.execute_plan(plan)
    
    def hold(self):
        global sim
        if sim:
            rospy.sleep(2)
        else:
            self.mgpi.open_gripper()
            rospy.sleep(2)
            self.mgpi.close_gripper()
            rospy.sleep(2)
    
    def release(self):
        if sim:
            rospy.sleep(2)
        else:
            rospy.sleep(2)
            self.mgpi.open_gripper()
            rospy.sleep(2)

    ### move a piece from one square to another
    def move_piece(self,start,end):
        
        #move the arm to home position 
        self.move2home()
        if sim:
            pass
        else:
            self.mgpi.open_gripper()
        #get the x,y of start and end positions
        start_pose= self.make_pose(self.square2xy(start))
        end_pose = self.make_pose(self.square2xy(end))
        print("Moving to start point")
        #Pick the cube from start pose
        self.move_arm(start_pose)
        time.sleep(1)
        self.hold()
        #self.mgpi.close_gripper
        #move back up to 20cm above
        self.move_up()
        time.sleep(0.5)
        print("Moving to goal point")
        #Place the cube at goal
        self.move_arm(end_pose)
        self.release()
        print('Move Complete')
        time.sleep(1)
        print('Returning Home')
        self.move2home()
        
    def attack_piece(self,start,end):
        self.move_piece(end,self.remove_position)
        self.move_piece(start,end)

    def spawn_piece(self,piece,square):
        
        default_square = 'D9'
        piece = piece.upper()
        if piece=='KING':
            pass
        elif piece=='QUEEN':
            pass
        elif piece=='BISHOP':
            pass
        elif piece=='PAWN':
            pass
        elif piece=='KNIGHT':
            pass    
        elif piece=='ROOK':
            pass
        else:
            piece_square = default_square

        self.move_piece(piece_square,square)
##Defining object class for the blocks to store its pose and id
class Block:
    def __init__(self,pose,id) :
        self.pose = pose
        self.id   = id


   

def main():
    
    try:
        arm = chess_arm()

        input("Initiate Demo..")
        print('Moving Home')
        arm.move2home()

        arm.move_arm(arm.make_pose(arm.square2xy('A8')))
        input("Position Board......")
        arm.move2home()
        
        input("Enter to Continue...")
        start = 'D7'
        end =  'D5'
        print("Moving from {} to {}".format(start,end))
        arm.move_piece(start,end)

        input("Enter to Continue..")
        start = 'C8'
        end =  'G4'
        print("Moving from {} to {}".format(start,end))
        arm.move_piece(start,end)

        
        input("Enter to Continue..")
        start = 'G4'
        end =  'E2'
        print("Moving from {} to {}".format(start,end))
        arm.attack_piece(start,end)

        input("Enter to Continue..")
        start = 'E2'
        end =  'D1'
        print("Moving from {} to {}".format(start,end))
        arm.attack_piece(start,end)

        input("Enter to Continue..")
        print("Spawning Queen")
        arm.spawn_piece('QUEEN','D8')
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
