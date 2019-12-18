#!/usr/bin/env python
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from duckie_rock.msg import awesome
from std_msgs.msg import Int16
import std_msgs
import math
import sys
import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

class sub_class:
	def __init__(self):
		self.sub = rospy.Subscriber("/processed_image_msg", awesome, self.callback, queue_size=1)		
		self.pub_control= rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
		self.rate = rospy.Rate(10) # 10hz
        	self.msg = awesome()

	def callback(self, msg):
		
		twist = Twist2DStamped()
		
		
		if math.fabs(msg.x_go-320) < 20:
			v=0.4
			w=0
			print "nono"	
			#twist.v = .4
			#twist.omega = 0
				
		elif (msg.x_go-320)<-20 and(msg.x_go-320)>-320 :
			v=0.3
			w=0.8
			print "haha"
			#twist.v = .2
			#twist.omega = 0.5
		elif (msg.x_go-320)>20 :
			v=0.2
			w=-0.5
			#twist.v = .2
			#twist.omega = -0.5
		else:
			v=0.0
			w=0.0	
		
		print (v,w)
		






if __name__ == "__main__":
    rospy.init_node("sub_node", anonymous=False)
    sub_node = sub_class()
    rospy.spin()
