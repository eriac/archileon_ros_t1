#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math

import os
import sys
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')
import getWayPoints
import getNearestPoint
import getInterSectionPoint


def br_rot_position_callback(float_msg):
    global world_br_rot_x
    global world_br_rot_y
    global world_br_rot_theta

    world_br_rot_x = float_msg.data[0]
    world_br_rot_y = float_msg.data[1]
    world_br_rot_theta = float_msg.data[2]


def br_tube_position_callback(float_msg):
    global world_br_tube_x
    global world_br_tube_y
    global world_br_tube_theta

    world_br_tube_x = float_msg.data[0]
    world_br_tube_y = float_msg.data[1]
    world_br_tube_theta = float_msg.data[2]


def rob_position_callback(float_msg):
    global world_rob_theta
    world_rob_theta = float_msg.data[2]


def br_way_points_callback(float_msg):
    global br_way_points
    br_way_points = []
    p_list = float_msg.data
    for index in range(0, len(p_list), 2):
        x = p_list[index]
        y = p_list[index+1]
        xy_list = []
        xy_list.append(x)
        xy_list.append(y)

        br_way_points.append(xy_list)


array = []
map_intersec = Float32MultiArray(data=array)
br_way_points = []

world_br_rot_x = None
world_br_rot_y = None
world_br_tube_x = None
world_br_tube_y = None
world_rob_theta = None

rospy.init_node("br_tube_inter_sec")

sub_br_way_points = rospy.Subscriber(
    'br_way_points', Float32MultiArray, br_way_points_callback)

sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)
sub_br_tube_status = rospy.Subscriber(
    "br_tube_status", Float32MultiArray, br_tube_position_callback)
sub_br_rot_status = rospy.Subscriber(
    "br_rot_status", Float32MultiArray, br_rot_position_callback)
pub_intersec_pos = rospy.Publisher(
    'br_intersec_status', Float32MultiArray, queue_size=1000)


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    print(len(br_way_points))

    if world_br_rot_x is not None and world_br_rot_y is not None and world_br_tube_x is not None and world_br_tube_y is not None and br_way_points != []:
        base_num = getNearestPoint.search_value_tube(
            br_way_points,
            world_br_tube_x,
            world_br_tube_y
        )

        w_result = getInterSectionPoint.calLine(
            center_x=world_br_rot_x,
            center_y=world_br_rot_y,
            p1_x=br_way_points[base_num][0],
            p1_y=br_way_points[base_num][1],
            p2_x=br_way_points[base_num + 1][0],
            p2_y=br_way_points[base_num + 1][1]
        )

        if len(w_result) == 2:
            inter_sec_x1 = w_result[0].x
            inter_sec_y1 = w_result[0].y

            inter_sec_x2 = w_result[1].x
            inter_sec_y2 = w_result[1].y

            diff1 = math.sqrt(
                ((world_br_tube_x - inter_sec_x1)**2 + (world_br_tube_y - inter_sec_y1)**2))
            diff2 = math.sqrt(
                ((world_br_tube_x - inter_sec_x2)**2 + (world_br_tube_y - inter_sec_y2)**2))

            if diff1 < diff2:
                w_intersec_x = inter_sec_x1
                w_intersec_y = inter_sec_y1
            else:
                w_intersec_x = inter_sec_x2
                w_intersec_y = inter_sec_y2

            map_intersec.data.append(w_intersec_x)
            map_intersec.data.append(w_intersec_y)
            pub_intersec_pos.publish(map_intersec)
            del map_intersec.data[:]

    rate.sleep()
