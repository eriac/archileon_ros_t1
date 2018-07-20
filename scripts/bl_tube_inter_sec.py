#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import getWayPoints
import getTubePosition
import getTubeAngle
import getInterSectionTube
import getNearestPoint
import getInterSectionPoint
import rotate_world_to_rob


def bl_rot_position_callback(float_msg):
    global world_bl_rot_x
    global world_bl_rot_y
    global world_bl_rot_theta

    world_bl_rot_x = float_msg.data[0]
    world_bl_rot_y = float_msg.data[1]
    world_bl_rot_theta = float_msg.data[2]


def bl_tube_position_callback(float_msg):
    global world_bl_tube_x
    global world_bl_tube_y
    global world_bl_tube_theta

    world_bl_tube_x = float_msg.data[0]
    world_bl_tube_y = float_msg.data[1]
    world_bl_tube_theta = float_msg.data[2]


def rob_position_callback(float_msg):
    global world_rob_theta
    world_rob_theta = float_msg.data[2]


array = []
map_intersec = Float32MultiArray(data=array)
bl_way_points = getWayPoints.read_bl_tube_points()


world_bl_rot_x = None
world_bl_rot_y = None
world_bl_tube_x = None
world_bl_tube_y = None
world_rob_theta = None

rospy.init_node("bl_tube_inter_sec")
sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)
sub_rob_status = rospy.Subscriber(
    "bl_tube_status", Float32MultiArray, bl_tube_position_callback)
sub_rob_status = rospy.Subscriber(
    "bl_rot_status", Float32MultiArray, bl_rot_position_callback)
pub_intersec_pos = rospy.Publisher(
    'bl_intersec_status', Float32MultiArray, queue_size=1000)


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    if world_bl_rot_x is not None and world_bl_rot_y is not None and world_bl_tube_x is not None and world_bl_tube_y is not None:
        base_num = getNearestPoint.search_value_tube(
            bl_way_points,
            world_bl_tube_x,
            world_bl_tube_y
        )
        w_result = getInterSectionPoint.calLine(
            center_x=world_bl_rot_x,
            center_y=world_bl_rot_y,
            p1_x=bl_way_points[base_num][0],
            p1_y=bl_way_points[base_num][1],
            p2_x=bl_way_points[base_num + 1][0],
            p2_y=bl_way_points[base_num + 1][1]
        )

        if len(w_result) == 2:
            inter_sec_x1 = w_result[0].x
            inter_sec_y1 = w_result[0].y

            inter_sec_x2 = w_result[1].x
            inter_sec_y2 = w_result[1].y

            diff1 = math.sqrt(
                ((world_bl_tube_x - inter_sec_x1)**2 + (world_bl_tube_y - inter_sec_y1)**2))
            diff2 = math.sqrt(
                ((world_bl_tube_x - inter_sec_x2)**2 + (world_bl_tube_y - inter_sec_y2)**2))

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
