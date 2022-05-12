#! /usr/bin/env python

import rospy
import geometry_msgs.msg
from move_group_interface import MoveGroupPythonInteface
from std_msgs.msg import Float32MultiArray
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def move_to_top(mgpi):
    """
    For test gripper in 2021, test picking up block 1
    """
    print("\n*************************************************")
    print("  4. Moving the end-effector in Cartesian space")
    print("*************************************************")
    print("- The desired pose is expressed using the geometry_msgs.msg.Pose() message.")
    print("- Position is a point in 3D space (x, y, z)")
    print("- Orientation is a quaternion")
    x = input("Input XXX")
    y = input("Input YYY")
    z = 0.086 # from last year exp
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.707
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.707
    mgpi.move_eef_to_pose(target_pose)
    

def main():
    try:
        mgpi = MoveGroupPythonInteface()  # Create and Initialize the interface
        print("Done.")
        for i in range(3):
            raw_input("Check block_1 pose and enter")  # waits for Enter
            mgpi.open_gripper()
            block_name = move_to_top(mgpi)  # Move to the top of block_1, manual input required
            mgpi.close_gripper("block_1") 

            raw_input("\nPress Enter to return back to home state.")# Move to home position and drop
            mgpi.move_to_joint_state(mgpi.home_joint_angles)
            mgpi.open_gripper("block_1")
            mgpi.deactivate_gripper()        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
