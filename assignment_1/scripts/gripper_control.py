#! /usr/bin/env python
#
# Author: Tejaswi Digumarti (tejaswi.digumarti@sydney.edu.au)
# Description: This code defines a class to control the Robotiq Gripper using ROS

import roslib
roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as gripperMsg


class GripperController(object):
    """
    Class to control the gripper
    """
    def __init__(self, stage):
        """
        Initializes the object.
        """
        super(GripperController, self).__init__()

        # rospy.init_node('GripperControllerNode')
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput',
                                   gripperMsg.Robotiq2FGripper_robot_output,
                                   queue_size=20)
        self.command = gripperMsg.Robotiq2FGripper_robot_output()
        self.stage = stage

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

        if object_to_detach is not None:
            self.stage.detach_object(object_to_detach)

    def close_gripper(self, object_to_attach=None):
        """
        Publishes the command to close the gripper
        """
        self.command.rPR = 255      # set gripper position to closed
        self.pub.publish(self.command)
        rospy.sleep(1)

        if object_to_attach is not None:
            self.stage.attach_object(object_to_attach)
