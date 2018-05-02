#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)

rate = rospy.Rate(10)
move_speed = 0.05
move_curve = 1 / 0.3

while not rospy.is_shutdown():
    # print(move_speed)
    pub_speed.publish(move_speed)
    pub_curve.publish(move_curve)
    rate.sleep()
