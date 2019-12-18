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
import imutils
import time
import os

class ssd_class:
    def __init__(self):
	self.pub_ssd = rospy.Publisher("ssd_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("usb_cam/image_raw/compressed", CompressedImage, self.ssd_image, queue_size=1)
        self.rate = rospy.Rate(10) # 10hz


    def image_filter(self, image):
	# initialize the list of class labels MobileNet SSD was trained to
	# detect, then generate a set of bounding box colors for each class
	CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
		"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
		"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
		"sofa", "train", "tvmonitor"]
	COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
	net = cv2.dnn.readNetFromCaffe(MobileNetSSD_deploy.prototxt.txt, MobileNetSSD_deploy.caffemodel)
	while True:
		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 400 pixels
		
		#frame = imutils.resize(image, width=400)
	 
		# grab the frame dimensions and convert it to a blob
		(h, w) = (460, 640)
		blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)),0.007843, (300, 300), 127.5)
	 
		# pass the blob through the network and obtain the detections and
		# predictions
		net.setInput(blob)
		detections = net.forward()
		# loop over the detections
		for i in np.arange(0, detections.shape[2]):
			# extract the confidence (i.e., probability) associated with
			# the prediction
			confidence = detections[0, 0, i, 2]
	 
			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > 0.2:
				# extract the index of the class label from the
				# `detections`, then compute the (x, y)-coordinates of
				# the bounding box for the object
				idx = int(detections[0, 0, i, 1])
				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")
	 
				# draw the prediction on the frame
				label = "{}: {:.2f}%".format(CLASSES[idx],confidence * 100)
				cv2.rectangle(image, (startX, startY), (endX, endY),COLORS[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				##cv2.putText(frame, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
	return image

    def ssd_image(self, image_msg):
	np_arr = np.fromstring(image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	frame = self.image_filter(image_np)	

	try:
		self.pub_ssd.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))  #8UC1
	except CvBridgeError as e:
		print(e)


if __name__ == "__main__":
    rospy.init_node("ssd_node", anonymous=False)
    ssd_node = ssd_class()
    rospy.spin()
