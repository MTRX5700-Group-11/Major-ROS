#code to get images from the realsense camera
#for mtrx5700 group-11 major project 2022
#partially based off code written for assignment 2 for this same assignment
#make sure to install the realsense ros library from https://github.com/IntelRealSense/realsense-ros
from ast import Pass
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

class StreamChessBoard():
    def __init__(self):
        sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)#create a subscriber to get realsense images from ROS
        sub_info =  rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)#create a subscriber to get realsense images from ROS
        self.bridge = CvBridge()
        self.delay = 0
        self.params = None
        self.coord = [[],[],[],[]]
        self.chess_board = None
        self.camera_stream = None
    def detect_april(self,image):
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

            # draw the bounding box of the AprilTag detection
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
            self.coord[tag_id]=(cX,cY)

        # show the output image after AprilTag detection
        return image

    def info_callback(self,info):
        global params
        K = info.K
        params = [K[0],K[4],K[2],K[5]]


    def image_callback(self,img_msg):

        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.camera_stream = image
        
        apr_img = self.detect_april(image.copy())

        cv2.imshow("Tag Detection", apr_img)
        thresh = 10
        crop_dim = 640

        #top left
        Ay = int(self.coord[3][1])
        Ax = int(self.coord[3][0])

        #top right
        By = int(self.coord[0][1])
        Bx = int(self.coord[0][0])

        #bottom left
        Cy = int(self.coord[1][1])
        Cx = int(self.coord[1][0])
        
        #bottom right
        Dy = int(self.coord[2][1])
        Dx = int(self.coord[2][0])
        
        roi_corners = np.float32([[Ax,Ay],[Bx,By],[Cx,Cy],[Dx,Dy]])
        resize_corners =np.float32([[0,0],[crop_dim,0],[0,crop_dim],[crop_dim,crop_dim]])

        matrix = cv2.getPerspectiveTransform(roi_corners,resize_corners)
        cropped_image = cv2.warpPerspective(image,matrix,(640,640))
        self.chess_board = cropped_image

        cv2.imshow("cropped_image",cropped_image)
        cv2.waitKey(20)
        
        

    

def image_getter():
    rospy.init_node('image_streamer', anonymous=True)

    rate = rospy.Rate(0.2)
    stream = StreamChessBoard()

    while not rospy.is_shutdown():
        pass
    rospy.spin()

if __name__ == '__main__':
    image_getter()

