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
import getTubeAngle
import rotate_world_to_rob


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
    global world_rob_x
    global world_rob_y
    global world_rob_theta
    world_rob_x = float_msg.data[0]
    world_rob_y = float_msg.data[1]
    world_rob_theta = float_msg.data[2]


def intersec_callback(float_msg):
    global world_inter_sec_x
    global world_inter_sec_y
    world_inter_sec_x = float_msg.data[0]
    world_inter_sec_y = float_msg.data[1]


r_br_rot_point_x = -0.1258
r_br_rot_point_y = -0.075
r_br_tube_point_x = -0.235
r_br_tube_point_y = -0.075

vec_r_br_rot_to_br_tube_x = r_br_tube_point_x - r_br_rot_point_x
vec_r_br_rot_to_br_tube_y = r_br_tube_point_y - r_br_rot_point_y


world_br_rot_x = None
world_br_rot_y = None
world_br_rot_theta = None
world_br_tube_x = None
world_br_tube_y = None
world_br_tube_theta = None
world_rob_x = None
world_rob_y = None
world_rob_theta = None
world_inter_sec_x = None
world_inter_sec_y = None


rospy.init_node("br_tube_angle_driver")
sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)
sub_rob_status = rospy.Subscriber(
    "br_tube_status", Float32MultiArray, br_tube_position_callback)
sub_rob_status = rospy.Subscriber(
    "br_rot_status", Float32MultiArray, br_rot_position_callback)
pub_intersec_pos = rospy.Subscriber(
    'br_intersec_status', Float32MultiArray, intersec_callback)
pub_br_tube_angle = rospy.Publisher(
    'br_rot_tube_angle', Float32, queue_size=1000)


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    if world_br_rot_x is not None and world_br_rot_y is not None and world_br_rot_theta is not None and world_br_tube_x is not None and world_br_tube_y is not None and world_rob_theta is not None and world_inter_sec_x is not None and world_inter_sec_y is not None:

        vector_r_br_rot_to_intersec = rotate_world_to_rob.cal(
            world_origin_x=world_br_rot_x,
            world_origin_y=world_br_rot_y,
            world_origin_theta=world_rob_theta,
            world_target_x=world_inter_sec_x,
            world_target_y=world_inter_sec_y
        )

        vector_r_br_rot_to_br_tube = rotate_world_to_rob.cal(
            world_origin_x=world_br_rot_x,
            world_origin_y=world_br_rot_y,
            world_origin_theta=world_rob_theta,
            world_target_x=world_br_tube_x,
            world_target_y=world_br_tube_y
        )

        radian = getTubeAngle.cal(
            u_x=vec_r_br_rot_to_br_tube_x,
            u_y=vec_r_br_rot_to_br_tube_y,
            v_x=vector_r_br_rot_to_intersec[0],
            v_y=vector_r_br_rot_to_intersec[1]
        )
        # print("degrees " + str(math.degrees(radian)))
        # print("radian " + str(radian) + "\n")

        if abs(radian) < math.pi / 4:
            pub_br_tube_angle.publish(radian)

        rate.sleep()
