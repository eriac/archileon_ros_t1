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


# r_bl_rot_point_x = -0.1258
# r_bl_rot_point_y = 0.075

# r_bl_tube_point_x = -0.235
# r_bl_tube_point_y = 0.075

# vector_r_bl_rot_to_bl_tube_x = r_bl_tube_point_x - r_bl_rot_point_x
# vector_r_bl_rot_to_bl_tube_y = r_bl_tube_point_y - r_bl_rot_point_y

radian_list = []


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
    theta = 0


class func_world_rob_pos():
    x = 0
    y = 0
    theta = 0


class func_param():
    bl_way_points = getWayPoints.read_bl_tube_points()
    counter = 0


rospy.init_node("vive_bl_tube_operation")

sub_rob_status = rospy.Subscriber(
    "/vive/bl_rot_status", Float32MultiArray, bl_rot_position_callback)

sub_rob_status = rospy.Subscriber(
    "/vive/bl_pos_status", Float32MultiArray, bl_pos_position_callback)

sub_rob_status = rospy.Subscriber(
    "/vive/LHR_1CDCEA0B", Float32MultiArray, rob_position_callback)


pub_bl_tube_angle = rospy.Publisher(
    'bl_rot_tube_angle', Float32, queue_size=1000)
pub_intersec_pos = rospy.Publisher(
    'intersec_status', Float32MultiArray, queue_size=1000)


rate = rospy.Rate(100)

radian_last = 0
while not rospy.is_shutdown():
    w_bl_rot_x = float(func_world_bl_rot_pos.x)
    w_bl_rot_y = float(func_world_bl_rot_pos.y)

    w_bl_tube_x = float(func_world_bl_tube_pos.x)
    w_bl_tube_y = float(func_world_bl_tube_pos.y)

    w_rob_pos_x = float(func_world_rob_pos.x)
    w_rob_pos_y = float(func_world_rob_pos.y)
    w_rob_theta = float(func_world_rob_pos.theta)

    print(float(func_world_bl_rot_pos.x))
    print(w_bl_rot_y)
    print(w_rob_theta)

    base_num = getNearestPoint.search_value_tube(
        func_param.bl_way_points,
        w_bl_tube_x,
        w_bl_tube_y
    )

    w_result = getInterSectionPoint.calLine(
        center_x=w_bl_rot_x,
        center_y=w_bl_rot_y,
        p1_x=func_param.bl_way_points[base_num][0],
        p1_y=func_param.bl_way_points[base_num][1],
        p2_x=func_param.bl_way_points[base_num + 1][0],
        p2_y=func_param.bl_way_points[base_num + 1][1]
    )

    if len(w_result) == 2:

        # print("inter_p " +str(inter_p[0]) + str(inter_p[1]))

        vector_r_bl_tube_to_intersec = rotate_world_to_rob.cal(
            world_origin_x=w_bl_tube_x,
            world_origin_y=w_bl_tube_y,
            world_origin_theta=w_rob_theta,
            world_target_x=float(w_result[0].x),
            world_target_y=float(w_result[0].y)
        )
    # print("vector_r_origin_to_intersec "+str(vector_r_origin_to_intersec))
        print("vector_r_bl_tube_to_intersec " +
              str(vector_r_bl_tube_to_intersec))

        vector_r_bl_tube_to_intersec2 = rotate_world_to_rob.cal(
            world_origin_x=w_bl_tube_x,
            world_origin_y=w_bl_tube_y,
            world_origin_theta=w_rob_theta,
            world_target_x=float(w_result[1].x),
            world_target_y=float(w_result[1].y)
        )

        print("vector_r_bl_tube_to_intersec2 " +
              str(vector_r_bl_tube_to_intersec2))

        diff1 = math.sqrt(
            (vector_r_bl_tube_to_intersec[0]**2 + vector_r_bl_tube_to_intersec[1]**2))
        diff2 = math.sqrt(
            (vector_r_bl_tube_to_intersec2[0]**2 + vector_r_bl_tube_to_intersec2[1]**2))

        print("diff1 " + str(diff1))
        print("diff2 " + str(diff2))

        if diff1 < diff2:
            w_intersec_x = float(w_result[0].x)
            w_intersec_y = float(w_result[0].y)
            print("AAAAAA")
        else:
            w_intersec_x = float(w_result[1].x)
            w_intersec_y = float(w_result[1].y)
            print("BBBBBB")

        print("w_intersec_x "+str(w_intersec_x))
        print("w_intersec_y "+str(w_intersec_y))

        # Visualize InterSection Point
        map_intersec.data.append(w_intersec_x)
        map_intersec.data.append(w_intersec_y)
        pub_intersec_pos.publish(map_intersec)
        del map_intersec.data[:]

        vector_r_bl_rot_to_intersec = rotate_world_to_rob.cal(
            world_origin_x=w_bl_rot_x,
            world_origin_y=w_bl_rot_y,
            world_origin_theta=w_rob_theta,
            world_target_x=w_intersec_x,
            world_target_y=w_intersec_y
        )

        vector_r_bl_rot_to_bl_tube = rotate_world_to_rob.cal(
            world_origin_x=w_bl_rot_x,
            world_origin_y=w_bl_rot_y,
            world_origin_theta=w_rob_theta,
            world_target_x=w_bl_tube_x,
            world_target_y=w_bl_tube_y
        )

        radian = getTubeAngle.cal(
            u_x=vector_r_bl_rot_to_bl_tube[0],
            u_y=vector_r_bl_rot_to_bl_tube[1],
            v_x=vector_r_bl_rot_to_intersec[0],
            v_y=vector_r_bl_rot_to_intersec[1]
        )
        print("degrees " + str(math.degrees(radian)))
        print("radian " + str(radian))

        pub_bl_tube_angle.publish(radian)

    # pub_bl_tube_angle.publish(radian*1.0+radian_last)
    # radian_last=radian*1.0+radian_last

    rate.sleep()
