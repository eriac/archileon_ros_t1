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

r_bl_tube_point_x = -0.235
r_bl_tube_point_y = 0.075

vector_r_bl_rot_to_bl_tube_x = r_bl_rot_point_x - r_bl_tube_point_x
vector_r_bl_rot_to_bl_tube_y = r_bl_rot_point_y - r_bl_tube_point_y

radian_list=[]

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

radian_last=0
while not rospy.is_shutdown():
    w_bl_rot_x = float(func_world_bl_rot_pos.x)
    w_bl_rot_y = float(func_world_bl_rot_pos.y)
    # w_bl_rot_theta = float(func_world_bl_rot_pos.theta)

    w_bl_tube_x = float(func_world_bl_tube_pos.x)
    w_bl_tube_y = float(func_world_bl_tube_pos.y)
    # w_bl_tube_theta = float(func_world_bl_tube_pos.theta)

    w_rob_pos_x = float(func_world_rob_pos.x)
    w_rob_pos_y = float(func_world_rob_pos.y)
    w_rob_theta = float(func_world_rob_pos.theta)

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

    if w_result:
        # diff_x = w_bl_tube_x - float(w_result[0].x)
        # diff_y = w_bl_tube_y - float(w_result[0].y)
        # diff1_xy = math.sqrt(diff_x**2 + diff_y**2)
        # print("diff1_xy " +str(diff1_xy))

        # diff_x = w_bl_tube_x - float(w_result[1].x)
        # diff_y = w_bl_tube_y - float(w_result[1].y)
        # diff2_xy = math.sqrt(diff_x**2 + diff_y**2)
        # print("diff2_xy " +str(diff2_xy))


        # if diff1_xy < diff2_xy:
        #     w_intersec_x = float(w_result[0].x)
        #     w_intersec_y = float(w_result[0].y)
        #     print("AAAAAAA")
        # else:
        #     w_intersec_x = float(w_result[1].x)
        #     w_intersec_y = float(w_result[1].y)
        #     print("BBBBBBB")
        for inter_p in w_result:  
            vector_r_bl_rot_to_intersec = rotate_world_to_rob.cal(
                world_origin_x=w_bl_rot_x,
                world_origin_y=w_bl_rot_y,
                world_origin_theta=w_rob_theta,
                world_target_x=float(inter_p[0]),
                world_target_y=float(inter_p[1])
            )
            radian = getTubeAngle.cal(
                u_x=vector_r_bl_rot_to_intersec[0],
                u_y=vector_r_bl_rot_to_intersec[1],
                v_x=vector_r_bl_rot_to_bl_tube_x,
                v_y=vector_r_bl_rot_to_bl_tube_y
            )
            # print("radian " +str(radian))
            if radian < math.pi/2:
                print("degrees " +str(math.degrees(radian)))
            # if abs(radian) > math.pi/2:
            #     print(inter_p[0])          
            #     print(inter_p[1])
            #     print("radian " + str(radian) +"\n")
                pub_bl_tube_angle.publish(radian)



            #     # Visualize InterSection Point
            #     map_intersec.data.append(inter_p[0])
            #     map_intersec.data.append(inter_p[1])
            #     pub_intersec_pos.publish(map_intersec)
            #     del map_intersec.data[:]


        # vector_r_bl_rot_to_bl_tube = rotate_world_to_rob.cal(
        #     world_origin_x=w_bl_rot_x,
        #     world_origin_y=w_bl_rot_y,
        #     world_origin_theta=w_rob_theta,
        #     world_target_x=w_bl_tube_x,
        #     world_target_y=w_bl_tube_y
        # )



        # pub_bl_tube_angle.publish(radian*1.0+radian_last)
        # radian_last=radian*1.0+radian_last

    rate.sleep()
