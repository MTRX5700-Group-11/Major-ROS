#code to get images from the realsense camera
#for mtrx5700 group-11 major project 2022
#partially based off code written for assignment 2 for this same assignment
#make sure to install the realsense ros library from https://github.com/IntelRealSense/realsense-ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import PIL
import sys
import time
output_file_path = "output_images/image"
bridge = CvBridge()
last_time = 0
delay = 30#delay in seconds
image_counter = 0

def image_callback(img_msg):
    #This section based off the 1st answer of this stackoverflow post https://stackoverflow.com/questions/69718148/access-image-via-opencv-in-python-ros-kinetic
    #rospy.loginfo(rospy.get_caller_id() + "Image recieved")
    #np_arr = np.fromstring(img_msg.data, np.uint8)
    #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    #stack-exchange based section ends here
    #image = cv2.flip(image,flipCode=0)
    cv2.imshow("realsense view",image)
    global last_time
    global image_counter
    cv2.waitKey(20)#human eye cannot real tell refresh faster than 50hz(note camera feed is 20hz, but set higher to allow running the rosbag faster)
    #store an image every 30 seconds
    if(time.time()>(last_time+30)):
        print('new image captured at ', time.time())
        last_time = time.time()
        image_path = output_file_path + str(image_counter) + ".png"
        cv2.imwrite(image_path,image)
        image_counter += 1


    

    

def image_getter():
    rospy.init_node('image_streamer', anonymous=True)
    sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)#create a subscriber to get realsense images from ROS
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    image_getter()

