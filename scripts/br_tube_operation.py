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


array = []
map_intersec = Float32MultiArray(data=array)


r_br_rot_point_x = -0.1258
r_br_rot_point_y = -0.075

r_br_tube_point_x = -0.235
r_br_tube_point_y = -0.075

vec_r_br_rot_to_br_tube_x = r_br_tube_point_x - r_br_rot_point_x
vec_r_br_rot_to_br_tube_y = r_br_tube_point_y - r_br_rot_point_y


def br_rot_position_callback(float_msg):
    func_world_br_rot_pos.x = float_msg.data[0]
    func_world_br_rot_pos.y = float_msg.data[1]
    func_world_br_rot_pos.theta = float_msg.data[2]


def br_pos_position_callback(float_msg):
    func_world_br_tube_pos.x = float_msg.data[0]
    func_world_br_tube_pos.y = float_msg.data[1]
    func_world_br_tube_pos.theta = float_msg.data[2]


def rob_position_callback(float_msg):
    func_world_rob_pos.x = float_msg.data[0]
    func_world_rob_pos.y = float_msg.data[1]
    func_world_rob_pos.theta = float_msg.data[2]


class func_world_br_rot_pos():
    x = 0
    y = 0
    theta = 0


class func_world_br_tube_pos():
    x = 0
    y = 0
    theta = 0


class func_world_rob_pos():
    x = 0
    y = 0
    theta = 0


class func_param():
    br_way_points = getWayPoints.read_br_tube_points()
    counter = 0


rospy.init_node("br_tube_operation")


sub_rob_status = rospy.Subscriber(
    "br_pos_status", Float32MultiArray, br_pos_position_callback)

sub_rob_status = rospy.Subscriber(
    "br_rot_status", Float32MultiArray, br_rot_position_callback)


sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)


pub_br_tube_angle = rospy.Publisher(
    'br_rot_tube_angle', Float32, queue_size=1000)

pub_intersec_pos = rospy.Publisher(
    'intersec_status', Float32MultiArray, queue_size=1000)


rate = rospy.Rate(100)

radian_last = 0
while not rospy.is_shutdown():
    w_br_rot_x = float(func_world_br_rot_pos.x)
    w_br_rot_y = float(func_world_br_rot_pos.y)

    w_br_tube_x = float(func_world_br_tube_pos.x)
    w_br_tube_y = float(func_world_br_tube_pos.y)

    w_rob_pos_x = float(func_world_rob_pos.x)
    w_rob_pos_y = float(func_world_rob_pos.y)

    w_rob_theta = float(func_world_rob_pos.theta)

    base_num = getNearestPoint.search_value_tube(
        func_param.br_way_points,
        w_br_tube_x,
        w_br_tube_y
    )

    print("base_num " + str(func_param.br_way_points[base_num]))
    print("next_num " + str(func_param.br_way_points[base_num + 1]))

    w_result = getInterSectionPoint.calLine(
        center_x=w_br_rot_x,
        center_y=w_br_rot_y,
        p1_x=func_param.br_way_points[base_num][0],
        p1_y=func_param.br_way_points[base_num][1],
        p2_x=func_param.br_way_points[base_num + 1][0],
        p2_y=func_param.br_way_points[base_num + 1][1]
    )

    if len(w_result) == 2:

        inter_sec_x1 = float(w_result[0].x)
        inter_sec_y1 = float(w_result[0].y)

        inter_sec_x2 = float(w_result[1].x)
        inter_sec_y2 = float(w_result[1].y)

        print("w_br_rot " + str(w_br_rot_x) + ", " + str(w_br_rot_y))
        print("w_br_tube " + str(w_br_tube_x) + ", " + str(w_br_tube_y))

        print("inter_sec1 " + str(inter_sec_x1) + ", " + str(inter_sec_y1))
        print("inter_sec2 " + str(inter_sec_x2) + ", " + str(inter_sec_y2))

        vec_r_br_tube_to_intersec1 = rotate_world_to_rob.cal(
            world_origin_x=w_br_tube_x,
            world_origin_y=w_br_tube_y,
            world_origin_theta=w_rob_theta,
            world_target_x=inter_sec_x1,
            world_target_y=inter_sec_y1
        )
        # print("vector_r_origin_to_intersec "+str(vector_r_origin_to_intersec))
        print("vec_r_br_tube_to_intersec1 " +
              str(vec_r_br_tube_to_intersec1))

        vec_r_br_tube_to_intersec2 = rotate_world_to_rob.cal(
            world_origin_x=w_br_tube_x,
            world_origin_y=w_br_tube_y,
            world_origin_theta=w_rob_theta,
            world_target_x=inter_sec_x2,
            world_target_y=inter_sec_y2
        )

        print("vec_r_br_tube_to_intersec2 " +
              str(vec_r_br_tube_to_intersec2))

        diff1 = math.sqrt(
            (vec_r_br_tube_to_intersec1[0]**2 + vec_r_br_tube_to_intersec1[1]**2))
        diff2 = math.sqrt(
            (vec_r_br_tube_to_intersec2[0]**2 + vec_r_br_tube_to_intersec2[1]**2))

        print("diff1 " + str(diff1))
        print("diff2 " + str(diff2))

        if diff1 < diff2:
            w_intersec_x = inter_sec_x1
            w_intersec_y = inter_sec_y1
            print("AAAAAA")
        else:
            w_intersec_x = inter_sec_x2
            w_intersec_y = inter_sec_y2
            print("BBBBBB")

        print("w_intersec_x "+str(w_intersec_x))
        print("w_intersec_y "+str(w_intersec_y))

        # Visualize InterSection Point
        map_intersec.data.append(w_intersec_x)
        map_intersec.data.append(w_intersec_y)
        pub_intersec_pos.publish(map_intersec)
        del map_intersec.data[:]

        vector_r_br_rot_to_intersec = rotate_world_to_rob.cal(
            world_origin_x=w_br_rot_x,
            world_origin_y=w_br_rot_y,
            world_origin_theta=w_rob_theta,
            world_target_x=w_intersec_x,
            world_target_y=w_intersec_y
        )

        vector_r_br_rot_to_br_tube = rotate_world_to_rob.cal(
            world_origin_x=w_br_rot_x,
            world_origin_y=w_br_rot_y,
            world_origin_theta=w_rob_theta,
            world_target_x=w_br_tube_x,
            world_target_y=w_br_tube_y
        )

        radian = getTubeAngle.cal(
            u_x=vec_r_br_rot_to_br_tube_x,
            u_y=vec_r_br_rot_to_br_tube_y,
            v_x=vector_r_br_rot_to_intersec[0],
            v_y=vector_r_br_rot_to_intersec[1]
        )
        print("degrees " + str(math.degrees(radian)))
        print("radian " + str(radian) + "\n")

        if abs(radian) < math.pi / 2:
            pub_br_tube_angle.publish(radian)

    # pub_br_tube_angle.pubrish(radian*1.0+radian_last)
    # radian_last = radian*1.0+radian_last

    rate.sleep()
