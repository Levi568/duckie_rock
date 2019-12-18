#!/usr/bin/env python
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int16
import std_msgs
import math
import sys
import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

#Define HSV colour range for certatn colour objects
yellowLower = (20,110,120)
yellowUpper = (45,255,255)
orangeLower = (10,100, 50)
orangeUpper = (25, 255, 255)

blueLower = (100,150, 0)
blueUpper = (140, 255, 255)

buffer = 20
pts = deque(maxlen = buffer)
counter = 0
(dX, dY) = (0, 0)

class cv_class:
    def __init__(self):
        self.pub_orange = rospy.Publisher("orange_image", Image, queue_size=1)
    	self.pub_x_coor = rospy.Publisher("image_center_x", Int16, queue_size=1)
	self.pub_y_coor = rospy.Publisher("image_center_y", Int16, queue_size=1)
	self.pub_radius = rospy.Publisher("object_radius", Int16, queue_size=1)

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, self.cv_image, queue_size=1)
        self.rate = rospy.Rate(10) # 10hz

    # color segmentation
    def image_filter(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_filter = cv2.inRange(hsv, blueLower, blueUpper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        orange_erode = cv2.erode(orange_filter, kernel, iterations=2)
        orange_dilate = cv2.dilate(orange_erode, kernel, iterations=2)

        #Find all contours in the masked image
        _,cnts,_ = cv2.findContours(orange_dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #Define center of the ball to be detected as None
        center = None
        x_coor = None
        y_coor = None
        radius = None

        #If any object is detected, then only proceed
        if(len(cnts) > 0):
            #Find the contour with maximum area
            c = max(cnts, key = cv2.contourArea)
            #Find the center of the circle, and its radius of the largest detected contour.
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            #Calculate the centroid of the ball, as we need to draw a circle around it.
            M = cv2.moments(c)

            x_coor = int(M['m10'] / M['m00'])
    	    y_coor = int(M['m01'] / M['m00'])
            center = (x_coor, y_coor)

            #Proceed only if a ball of considerable size is detected
            if radius > 5: #draw the circle and the center
                cv2.circle(image, (int(x), int(y)), int(radius), (0,255,255), 2)
                cv2.circle(image, center, 5, (0,255,255), -1)
                #Append the detected object in the frame to pts deque structure
                pts.appendleft(center)

        #Using numpy arange function for better performance. Loop till all detected points
        for i in np.arange(1, len(pts)):
            #If no points are detected, move on.
            if(pts[i-1] == None or pts[i] == None):
                continue

            #If atleast 10 frames have direction change, proceed
            if counter >= 10 and i == 1 and pts[-10] is not None:
                #Calculate the distance between the current frame and 10th frame before
                dX = pts[-10][0] - pts[i][0]
                dY = pts[-10][1] - pts[i][1]
                (dirX, dirY) = ('', '')

                #If distance is greater than 100 pixels, considerable direction change has occured.
                if np.abs(dX) > 100:
                    dirX = 'West' if np.sign(dX) == 1 else 'East'

                if np.abs(dY) > 100:
                    dirY = 'North' if np.sign(dY) == 1 else 'South'

                #Set direction variable to the detected direction
                direction = dirX if dirX != '' else dirY

            #Draw a trailing red line to depict motion of the object.
            thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
            cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)

        return image, center, x_coor, y_coor, radius

    def cv_image(self, image_msg):
        np_arr = np.fromstring(image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        color, center, x_coor, y_coor, radius = self.image_filter(image_np)
	print "object center:",center

	self.pub_x_coor.publish(x_coor)
	self.pub_y_coor.publish(y_coor)
	self.pub_radius.publish(radius)

        try:
            self.pub_orange.publish(self.bridge.cv2_to_imgmsg(color, "bgr8"))  #8UC1
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node("cv_node", anonymous=False)
    cv_node = cv_class()
    rospy.spin()
