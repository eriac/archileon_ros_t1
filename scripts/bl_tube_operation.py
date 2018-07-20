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
    global world_rob_x
    global world_rob_y
    global world_rob_theta

    world_rob_x = float_msg.data[0]
    world_rob_y = float_msg.data[1]
    world_rob_theta = float_msg.data[2]


array = []
map_intersec = Float32MultiArray(data=array)
bl_way_points = getWayPoints.read_bl_tube_points()

r_bl_rot_point_x = -0.1258
r_bl_rot_point_y = 0.075
r_bl_tube_point_x = -0.235
r_bl_tube_point_y = 0.075

vec_r_bl_rot_to_bl_tube_x = r_bl_tube_point_x - r_bl_rot_point_x
vec_r_bl_rot_to_bl_tube_y = r_bl_tube_point_y - r_bl_rot_point_y


world_bl_rot_x = None
world_bl_rot_y = None
world_bl_rot_theta = None

world_bl_tube_x = None
world_bl_tube_y = None
world_bl_tube_theta = None

world_rob_x = None
world_rob_y = None
world_rob_theta = None


rospy.init_node("bl_tube_operation")

sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)


sub_rob_status = rospy.Subscriber(
    "bl_tube_status", Float32MultiArray, bl_tube_position_callback)

sub_rob_status = rospy.Subscriber(
    "bl_rot_status", Float32MultiArray, bl_rot_position_callback)

pub_bl_tube_angle = rospy.Publisher(
    'bl_rot_tube_angle', Float32, queue_size=1000)

pub_intersec_pos = rospy.Publisher(
    'intersec_status', Float32MultiArray, queue_size=1000)


rate = rospy.Rate(100)

while not rospy.is_shutdown():

    if world_bl_rot_x is not None and world_bl_rot_y is not None and world_bl_rot_theta is not None and world_bl_tube_x is not None and world_bl_tube_y is not None and world_rob_theta is not None:

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

            vec_r_bl_tube_to_intersec1 = rotate_world_to_rob.cal(
                world_origin_x=world_bl_tube_x,
                world_origin_y=world_bl_tube_y,
                world_origin_theta=world_rob_theta,
                world_target_x=inter_sec_x1,
                world_target_y=inter_sec_y1
            )

            vec_r_bl_tube_to_intersec2 = rotate_world_to_rob.cal(
                world_origin_x=world_bl_tube_x,
                world_origin_y=world_bl_tube_y,
                world_origin_theta=world_rob_theta,
                world_target_x=inter_sec_x2,
                world_target_y=inter_sec_y2
            )

            diff1 = math.sqrt(
                (vec_r_bl_tube_to_intersec1[0]**2 + vec_r_bl_tube_to_intersec1[1]**2))
            diff2 = math.sqrt(
                (vec_r_bl_tube_to_intersec2[0]**2 + vec_r_bl_tube_to_intersec2[1]**2))

            if diff1 < diff2:
                w_intersec_x = inter_sec_x1
                w_intersec_y = inter_sec_y1
            else:
                w_intersec_x = inter_sec_x2
                w_intersec_y = inter_sec_y2

            # Visualize InterSection Point
            map_intersec.data.append(w_intersec_x)
            map_intersec.data.append(w_intersec_y)
            pub_intersec_pos.publish(map_intersec)
            del map_intersec.data[:]

            vector_r_bl_rot_to_intersec = rotate_world_to_rob.cal(
                world_origin_x=world_bl_rot_x,
                world_origin_y=world_bl_rot_y,
                world_origin_theta=world_rob_theta,
                world_target_x=w_intersec_x,
                world_target_y=w_intersec_y
            )

            vector_r_bl_rot_to_bl_tube = rotate_world_to_rob.cal(
                world_origin_x=world_bl_rot_x,
                world_origin_y=world_bl_rot_y,
                world_origin_theta=world_rob_theta,
                world_target_x=world_bl_tube_x,
                world_target_y=world_bl_tube_y
            )

            radian = getTubeAngle.cal(
                u_x=vec_r_bl_rot_to_bl_tube_x,
                u_y=vec_r_bl_rot_to_bl_tube_y,
                v_x=vector_r_bl_rot_to_intersec[0],
                v_y=vector_r_bl_rot_to_intersec[1]
            )
            print("degrees " + str(math.degrees(radian)))
            print("radian " + str(radian) + "\n")

            if abs(radian) < math.pi / 4:
                pub_bl_tube_angle.publish(radian)

        rate.sleep()
