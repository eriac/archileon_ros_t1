#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32MultiArray


rospy.init_node("talker")

pub_br_tube_angle = rospy.Publisher(
    'xy_list', Float32MultiArray, queue_size=1000)

array = []
point_list = Float32MultiArray(data=array)
point_list.data.append(2)
point_list.data.append(3)


rate = rospy.Rate(1)


while not rospy.is_shutdown():

    pub_br_tube_angle.publish(point_list)

    rate.sleep()
