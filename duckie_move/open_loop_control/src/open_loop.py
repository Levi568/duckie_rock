#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import Twist2DStamped

if __name__ == '__main__':
    rospy.init_node('open_loop', anonymous=True)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    t_start = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        msg = Twist2DStamped()
        dt = t - t_start
        if dt > 4 and dt < 5:
            msg.v = 0.55
            msg.omega = 0.0
        elif dt > 5 and dt < 8:
            msg.v = 0.15
            msg.omega = 2.0
        elif dt > 8 and dt < 9:
            msg.v = 0.55
            msg.omega = 0.0
        elif dt > 9 and dt < 11:
            msg.v = 0.15
            msg.omega = 2.0
        elif dt > 11 and dt < 12:
            msg.v = 0.55
            msg.omega = 0.0
        elif dt > 12 and dt < 15:
            msg.v = 0.15
            msg.omega = 2.0
        elif dt > 15 and dt < 16:
            msg.v = 0.55
            msg.omega = 0.0
        else:
            msg.v = 0
            msg.omega = 0
        pub.publish(msg)
        rate.sleep()
