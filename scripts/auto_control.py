#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

rospy.init_node("topic_pub")
pub = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub = rospy.Publisher('move_time', Float32, queue_size=1000)

rate = rospy.Rate(10)
move_speed = 0.05
move_curve = 0.3

while not rospy.is_shutdown():
    pub.publish(move_speed)
    pub.publish(move_curve)
    rate.sleep()
