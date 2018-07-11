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


r_bl_rot_point_x = -0.1258
r_bl_rot_point_y = 0.075


def bl_rot_position_callback(float_msg):
    func_world_bl_rot_pos.x = float_msg.data[0]
    func_world_bl_rot_pos.y = float_msg.data[1]
    func_world_bl_rot_pos.theta = float_msg.data[2]


def bl_pos_position_callback(float_msg):
    func_world_bl_tube_pos.x = float_msg.data[0]
    func_world_bl_tube_pos.y = float_msg.data[1]
    func_world_bl_tube_pos.theta = float_msg.data[2]


def rob_position_callback(float_msg):
    func_world_rob_pos.x = float_msg.data[0]
    func_world_rob_pos.y = float_msg.data[1]
    func_world_rob_pos.theta = float_msg.data[2]


class func_world_bl_rot_pos():
    x = 0
    y = 0
    theta = 0


class func_world_bl_tube_pos():
    x = 0
    y = 0


class func_world_rob_pos():
    x = 0
    y = 0
    theta = 0


class func_param():
    bl_way_points = getWayPoints.read_bl_tube_points()
    counter = 0


rospy.init_node("tube_operation")

sub_rob_status = rospy.Subscriber(
    "bl_rot_status", Float32MultiArray, bl_rot_position_callback)
sub_rob_status = rospy.Subscriber(
    "bl_pos_status", Float32MultiArray, bl_pos_position_callback)
sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)


pub_bl_tube_angle = rospy.Publisher(
    'bl_rot_tube_angle', Float32, queue_size=1000)

pub_intersec_pos = rospy.Publisher(
    'intersec_status', Float32MultiArray, queue_size=1000)


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    w_bl_rot_x = float(func_world_bl_rot_pos.x)
    w_bl_rot_y = float(func_world_bl_rot_pos.y)

    w_bl_tube_x = float(func_world_bl_tube_pos.x)
    w_bl_tube_y = float(func_world_bl_tube_pos.y)

    w_rob_pos_x = float(func_world_rob_pos.x)
    w_rob_pos_y = float(func_world_rob_pos.y)
    w_rob_theta = float(func_world_rob_pos.theta)

    base_num = getNearestPoint.search_value_tube(
        func_param.bl_way_points,
        w_bl_tube_x,
        w_bl_tube_y
    )

    # print("base_num " +str(base_num))
    # print("num " +str(base_num + 1))
    # print("way_points " +str(func_param.bl_way_points[base_num]))
    # print("num_points " +str(func_param.bl_way_points[base_num+1]))
    # print("w_rob_theta " +str(w_rob_theta))
    # print("r_bl_rot_point " +str(float(r_rob_to_bl_rot_point[0]))+ ", " +str(float(r_rob_to_bl_rot_point[1])))
    # print("r_rob_to_base_target " +str(r_rob_to_base_target))
    # print("r_rob_to_near_target " +str(r_rob_to_near_target))

    w_result = getInterSectionPoint.calLine(
        center_x=w_bl_rot_x,
        center_y=w_bl_rot_y,
        p1_x=func_param.bl_way_points[base_num][0],
        p1_y=func_param.bl_way_points[base_num][1],
        p2_x=func_param.bl_way_points[base_num + 1][0],
        p2_y=func_param.bl_way_points[base_num + 1][1]
    )

    if w_result:
        print("result1 " +
              str(float(w_result[0].x)) + ", " + str(float(w_result[0].y)))
        print("result2 " +
              str(float(w_result[1].x)) + ", " + str(float(w_result[1].y)))

        diff_x = w_bl_tube_x - float(w_result[0].x)
        diff_y = w_bl_tube_y - float(w_result[0].y)
        diff1_xy = math.sqrt(diff_x**2 + diff_y**2)

        diff_x = w_bl_tube_x - float(w_result[1].x)
        diff_y = w_bl_tube_y - float(w_result[1].y)
        diff2_xy = math.sqrt(diff_x**2 + diff_y**2)

        if diff1_xy < diff2_xy:
            w_intersec_x = float(w_result[0].x)
            w_intersec_y = float(w_result[0].y)
        else:
            w_intersec_x = float(w_result[1].x)
            w_intersec_y = float(w_result[1].y)

        print("w_intersec_x " + str(w_intersec_x))
        print("w_intersec_y " + str(w_intersec_y))

        vector_r_bl_rot_to_intersec = rotate_world_to_rob.cal(
            world_origin_x=w_bl_rot_x,
            world_origin_y=w_bl_rot_y,
            world_origin_theta=w_rob_theta,
            world_target_x=w_intersec_x,
            world_target_y=w_intersec_y
        )
        print("vector_r_bl_rot_to_intersec " +
              str(vector_r_bl_rot_to_intersec))

        vector_r_bl_rot_to_bl_tube = rotate_world_to_rob.cal(
            world_origin_x=w_bl_rot_x,
            world_origin_y=w_bl_rot_y,
            world_origin_theta=w_rob_theta,
            world_target_x=w_bl_tube_x,
            world_target_y=w_bl_tube_y
        )

        print("vector_r_bl_rot_to_bl_tube " +
              str(vector_r_bl_rot_to_bl_tube) + "\n")

        radian = getTubeAngle.cal(
            u_x=vector_r_bl_rot_to_intersec[0],
            u_y=vector_r_bl_rot_to_intersec[1],
            v_x=vector_r_bl_rot_to_bl_tube[0],
            v_y=vector_r_bl_rot_to_bl_tube[1]
        )

        print("radian " + str(radian))
        pub_bl_tube_angle.publish(radian)

        map_intersec.data.append(w_intersec_x)
        map_intersec.data.append(w_intersec_y)
        print("map_intersec.data "+str(map_intersec.data) + "\n")
        pub_intersec_pos.publish(map_intersec)
        del map_intersec.data[:]

    rate.sleep()
