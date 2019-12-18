#!/usr/bin/env python
from sensor_msgs.msg import Image, CompressedImage
import math
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

#Define HSV colour range for certatn colour objects
yellowLower = (20,110,120)
yellowUpper = (45,255,255)


class cv_class:
    def __init__(self):
        self.pub_orange = rospy.Publisher("orange_image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, self.hough_image, queue_size=1)

        self.rate = rospy.Rate(10) # 10hz

    # color segmentation
    def image_filter(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_filter = cv2.inRange(hsv, yellowLower, yellowUpper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        orange_dilate = cv2.dilate(orange_filter, kernel, iterations=1)
        #orange_erode = cv2.erode(orange_filter, kernel, iterations=1)

        
        #Find all contours in the masked image
        _,cnts,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #Define center of the ball to be detected as None
        center = None

        return orange_dilate

    def hough_image(self, image_msg):
        np_arr = np.fromstring(image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        orange = self.image_filter(image_np)

        try:
            self.pub_orange.publish(self.bridge.cv2_to_imgmsg(orange, "8UC1"))
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node("cv_node", anonymous=False)
    cv_node = cv_class()
    rospy.spin()
