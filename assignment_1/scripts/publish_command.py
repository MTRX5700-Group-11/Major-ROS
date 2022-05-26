#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from assignment_1.msg import arm_command


pub = rospy.Publisher('arm_command', arm_command, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
arm_command = arm_command()
arm_command.command = "Move"
arm_command.start = "A1"
arm_command.end = "A2"
while not rospy.is_shutdown():
   
   input("Press Enter")
   pub.publish(arm_command)
   r.sleep()