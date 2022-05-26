#! /usr/bin/env python

from cmath import pi
import time
from re import S
from shutil import move
from turtle import home
from moveit_python import MoveGroupInterface
import rospy
from move_group_interface import MoveGroupPythonInteface
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as gripperMsg
from assignment_1.msg import arm_command
from tf.transformations import *

class GripperController():
    """
    Class to control the gripper
    """
    def __init__(self):
        # rospy.init_node('GripperControllerNode')
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput',
                                   gripperMsg.Robotiq2FGripper_robot_output,
                                   queue_size=20)
        self.command = gripperMsg.Robotiq2FGripper_robot_output()

    def activate_gripper(self):
        """
        Publishes the command to activate the gripper and set its speed and force.
        """
        self.command.rACT = 1       # activate the gripper
        self.command.rGTO = 1
        self.command.rSP = 255      # sets the speed of the gripper
        self.command.rFR = 150      # sets the force that the gripper applies
        self.pub.publish(self.command)
        rospy.sleep(1)


    def deactivate_gripper(self):
        """
        Publishes the command to deactivate the gripper
        """
        self.command.rACT = 0       # deactivate gripper
        self.pub.publish(self.command)
        rospy.sleep(1)

    def open_gripper(self, object_to_detach=None):
        """
        Publishes the command to open the gripper
        """
        self.command.rPR = 0        # set gripper position to open
        self.pub.publish(self.command)
        rospy.sleep(1)


    def close_gripper(self, object_to_attach=None):
        """
        Publishes the command to close the gripper
        """
        self.command.rPR = 255      # set gripper position to closed
        self.pub.publish(self.command)
        rospy.sleep(1)



class chess_arm():
    def __init__(self):
        self.mgpi = MoveGroupPythonInteface() 
        self.gripper = GripperController()
        rospy.Subscriber("arm_command",arm_command,self.callback)
        self.gripper.activate_gripper()
        #Optimal home position
        self.home_state = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
        #setting the movement height to be 7cm to avoid any collision
        self.cube_height = 0.03
        #x,y position of A8 --> x = 0.21 ;y = 0.44 
        self.a8_position = [0.21,0.44]
        #square width in m
        self.square_width = 0.06
        self.remove_position = 'A0'
    
    def callback(self,data):
        command = data.command.upper()
        start = data.start.upper()
        end = data.end.upper()

        self.move_piece(start,end)


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

    
    def hold(self,pose):
        self.cube_height = -0.015
        self.move_static(pose)
        self.cube_height = 0.03
        self.gripper.open_gripper()
        rospy.sleep(2)
        self.gripper.close_gripper()
        rospy.sleep(2)
    
    def release(self,pose):
        rospy.sleep(2)
        self.cube_height = -0.015
        self.move_static(pose)
        self.cube_height = 0.03
        self.gripper.open_gripper()
        rospy.sleep(2)

    ### move a piece from one square to another
    def move_piece(self,start,end):
        
        #move the arm to home position 
        self.gripper.open_gripper()
        self.move2home()
        self.gripper.open_gripper()
        #get the x,y of start and end positions
        start_pose= self.make_pose(self.square2xy(start))
        end_pose = self.make_pose(self.square2xy(end))
        print("Moving to start point")
        #Pick the cube from start pose
        self.move_arm(start_pose)
        time.sleep(1)
        self.hold(start_pose)
        #self.gripper.close_gripper
        #move back up to 20cm above
        self.move_up()
        time.sleep(0.5)
        print("Moving to goal point")
        #Place the cube at goal
        self.move_arm(end_pose)
        self.release(end_pose)
        #self.open_gripper()
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
    # rospy.init_node('arm_control')
    # r = rospy.Rate(10) # 10hz
    try:
        arm = chess_arm()
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()