#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
from std_msgs.msg import Int32


import os
import sys
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')
import getWayPoints
import getNearestPoint
import getInterSectionPoint

w_bl_rot_x = None
w_bl_rot_y = None
w_bl_tube_x = None
w_bl_tube_y = None
fixed_base_num = None
counter = 0

array = []
map_intersec = Float32MultiArray(data=array)

bl_tube_way_points = "nozzle_in.txt"
bl_tube_way_points = getWayPoints.first_read(bl_tube_way_points)


def bl_rot_position_callback(float_msg):
    global w_bl_rot_x
    global w_bl_rot_y

    w_bl_rot_x = float_msg.data[0]
    w_bl_rot_y = float_msg.data[1]


def bl_tube_position_callback(float_msg):
    global w_bl_tube_x
    global w_bl_tube_y

    w_bl_tube_x = float_msg.data[0]
    w_bl_tube_y = float_msg.data[1]

    if counter == 0:
        global fixed_base_num
        fixed_base_num = getNearestPoint.search_value_tube(
            bl_tube_way_points,
            w_bl_tube_x,
            w_bl_tube_y
        )
        global counter
        counter += 1


rospy.init_node("bl_tube_find")

sub_bl_tube_status = rospy.Subscriber(
    "bl_tube_status", Float32MultiArray, bl_tube_position_callback)
sub_bl_rot_status = rospy.Subscriber(
    "bl_rot_status", Float32MultiArray, bl_rot_position_callback)
pub_intersec_pos = rospy.Publisher(
    'bl_fixed_base_num', Int32, queue_size=1000)


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    if w_bl_rot_x is not None and w_bl_rot_y is not None and w_bl_tube_x is not None and w_bl_tube_y is not None and fixed_base_num is not None:
        diff = getNearestPoint.get_dist_tube_to_points(
            forward_x=bl_tube_way_points[fixed_base_num + 1][0],
            forward_y=bl_tube_way_points[fixed_base_num + 1][1],
            world_tube_x=w_bl_tube_x,
            world_tube_y=w_bl_tube_y,
        )
        print(diff)
        if diff < 0.005:
            global fixed_base_num
            fixed_base_num += 1
        print(bl_tube_way_points[fixed_base_num + 1])
        pub_intersec_pos.publish(fixed_base_num)
    rate.sleep()