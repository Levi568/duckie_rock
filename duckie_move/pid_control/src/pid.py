#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, LanePose
import math
import sys
import rospy
import time
import numpy as np

class pid_class:
    def __init__(self):
        self.error_accum = 0
        self.previous_error_phi = 0
        self.previous_error_d = 0
        self.pub_control= rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.pid_motion, queue_size=1)

        self.rate = rospy.Rate(10) # 10hz
        self.lp_msg = LanePose()
        self.t_start = None
    def pid_motion(self, lp_msg):
        kp_phi = 4.0
        kd_phi = 0.0
        ki_phi = 1.0

        kp_d = 6.0
        kd_d = 0.0
        ki_d = 1.0

        t_prev = self.t_start
        twist = Twist2DStamped()

        t = rospy.get_time()
        if t_prev != None :

                dt = t - t_prev
                error_d = 0 - lp_msg.d   # 0 is the desired value of distance from the center
                error_phi = 0 - lp_msg.phi  # 0 is the desired value of angle

                self.error_accum += error_d * dt
                integral = self.error_accum

                if dt != 0:
                    derivative_d = (error_d - self.previous_error_d)/dt
                    derivative_phi = (error_phi - self.previous_error_phi)/dt
                else:
                    derivative_d = 0.0
                    derivative_phi = 0.0

                control_phi = kp_phi*error_phi + kd_phi*derivative_phi + ki_phi*0 # acceleration from engine
                control_phi = max(min(control_phi, 10.0), -10.0)
                control_d = kp_d*error_d + kd_d*derivative_d + ki_d*integral # acceleration from engine
                control_d = max(min(control_d, 10.0), -10.0)
                control = control_phi + control_d
                twist.omega = control
                twist.v = 0.3

                print("Error =", error_d, error_phi)
                print("Control", control)

                self.previous_error_phi = error_phi
                self.previous_error_d = error_d
                t_prev = t
                self.t_start = t

              self.t_start = t
              self.pub_control.publish(twist)

if __name__ == "__main__":
              rospy.init_node("pid_node", anonymous=False)  # adapted to sonjas default file
              pid_node = pid_class()
              rospy.spin()
