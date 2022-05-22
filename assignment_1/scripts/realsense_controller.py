#code to get images from the realsense camera
#for mtrx5700 group-11 major project 2022
#partially based off code written for assignment 2 for this same assignment
#make sure to install the realsense ros library from https://github.com/IntelRealSense/realsense-ros
from turtle import shape
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
import cv2
import apriltag
from cv_bridge import CvBridge
import PIL
import sys
import time
output_file_path = "output_images/image"
bridge = CvBridge()
last_time = 0
delay = 30#delay in seconds
image_counter = 0
capture = False
params = None
coord = [[],[]]

def detect_april(image):
    # camera parameters
    global params
    global coord
    tag_size = 0.06

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    options = apriltag.DetectorOptions(families="tag36h10")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    

    

    # loop over the AprilTag detection results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # print("Tag: {}".format(r.tag_id))
        # print("-----------")
        # print("Centre: {}".format(r.center))
        # print("-----------")
                # print("Corners: {}".format(r.corners))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tag_id = r.tag_id
        cv2.putText(image, str(tag_id), (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        coord[tag_id] = r.center
        # print("Pose of tag {} is {}".format(tag_id,pose[0]))
    # show the output image after AprilTag detection
    cv2.imshow("Tag Detection", image)

def info_callback(info):
    global params
    K = info.K
    params = [K[0],K[4],K[2],K[5]]


def image_callback(img_msg):
    #This section based off the 1st answer of this stackoverflow post https://stackoverflow.com/questions/69718148/access-image-via-opencv-in-python-ros-kinetic
    #rospy.loginfo(rospy.get_caller_id() + "Image recieved")
    #np_arr = np.fromstring(img_msg.data, np.uint8)
    #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    #stack-exchange based section ends here
    #image = cv2.flip(image,flipCode=0)
    detect_april(image.copy())
    global coord
    thresh = 10
    # x_min = int(coord[0][1])-thresh
    # x_max = int(coord[1][1])-thresh
    # y_min = int(coord[1][0])+thresh
    # y_max = int(coord[0][0])+thresh
    
    # print(x_min,x_max, y_min,y_max)
    cropped_image = image[20:670,120:770]
    cv2.imshow("cropped_image",cropped_image)
    cv2.imshow("realsense view",image)
    global capture
    global last_time
    global image_counter
    cv2.waitKey(20)#human eye cannot real tell refresh faster than 50hz(note camera feed is 20hz, but set higher to allow running the rosbag faster)
    #store an image every 30 seconds
    if(capture):
        print('\nnew image captured at ', time.time())
        last_time = time.time()
        image_path = output_file_path + str(image_counter) + ".png"
        cv2.imwrite(image_path,cropped_image)
        image_counter += 1
        print("Image Count: ",image_counter)
        capture = False


    

    

def image_getter():
    rospy.init_node('image_streamer', anonymous=True)
    global capture
    sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)#create a subscriber to get realsense images from ROS
    sub_image = rospy.Subscriber("/camera/color/camera_info", CameraInfo, info_callback)#create a subscriber to get realsense images from ROS
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        rate.sleep()
        if(input("Press Enter to Capture!\n")!=None):
            capture = True
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    image_getter()

