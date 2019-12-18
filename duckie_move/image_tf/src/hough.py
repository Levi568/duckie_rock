#!/usr/bin/env python
from sensor_msgs.msg import Image, CompressedImage
import math
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class hough_class:
    def __init__(self):
        self.pub_white = rospy.Publisher("hough_white_image", Image, queue_size=1)
        self.pub_yellow = rospy.Publisher("hough_yellow_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("~compressed", CompressedImage, self.image_filter, queue_size=1)

        self.rate = rospy.Rate(10) # 10hz

    def get_lines(self, original_image, filtered_image):
        r_res = 1
        theta_res = np.pi/180
        threshold = 6
        min_length = 3
        max_gap = 5
        lines = cv2.HoughLinesP(filtered_image, r_res, theta_res, threshold,np.empty(1), min_length, max_gap)
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                print(lines[i])
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv2.LINE_AA)
        return output

    def image_filter(self, image_msg):
	np_arr = np.fromstring(image_msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white_filter = cv2.inRange(hsv, (10,5,150), (255,50,255))
        yellow_filter = cv2.inRange(hsv, (20,120,80), (60,255,255))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        white_dilate = cv2.dilate(white_filter, kernel)
        yellow_dilate = cv2.dilate(yellow_filter, kernel)
        edges = cv2.Canny(image, 0, 300, apertureSize=3)
        white_edges = cv2.bitwise_and(white_dilate, edges)
        yellow_edges = cv2.bitwise_and(yellow_dilate, edges)
        white_output = self.get_lines(image, white_edges)
        
        try:
            self.pub_white.publish(self.bridge.cv2_to_imgmsg(white_output, "bgr8"))
            
        except CvBridgeError as e:
            print(e)
	
	yellow_output = self.get_lines(image, yellow_edges)
	try:
            
            self.pub_yellow.publish(self.bridge.cv2_to_imgmsg(yellow_output, "bgr8"))
        except CvBridgeError as e:
            print(e)



if __name__ == "__main__":
    rospy.init_node("hough_node", anonymous=False)
    hough_node = hough_class()
rospy.spin()
