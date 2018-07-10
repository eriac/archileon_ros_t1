#!/usr/bin/env python
#coding: UTF-8
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
    counter =0


rospy.init_node("tube_operation")

sub_rob_status = rospy.Subscriber("bl_rot_status", Float32MultiArray, bl_rot_position_callback)
sub_rob_status = rospy.Subscriber("bl_pos_status", Float32MultiArray, bl_pos_position_callback)
sub_rob_status = rospy.Subscriber("robot_status", Float32MultiArray, rob_position_callback)


pub_bl_tube_angle = rospy.Publisher('bl_rot_tube_angle', Float32, queue_size=1000)

pub_intersec_pos = rospy.Publisher('intersec_status', Float32MultiArray, queue_size=1000)



rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if func_param.counter ==0:
        pass
    else:
        w_rob_bl_rot_x = float(func_world_bl_rot_pos.x)
        w_rob_bl_rot_y = float(func_world_bl_rot_pos.y)
        
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
        for num in range(base_num-2, base_num+2):

            r_base_target = rotate_world_to_rob.cal(
                world_rob_x = w_rob_pos_x, 
                world_rob_y = w_rob_pos_y, 
                world_rob_theta = w_rob_theta, 
                world_target_x = func_param.bl_way_points[num][0], 
                world_target_y = func_param.bl_way_points[num][1]
            )

            r_near_target = rotate_world_to_rob.cal(
                world_rob_x = w_rob_pos_x, 
                world_rob_y = w_rob_pos_y, 
                world_rob_theta = w_rob_theta, 
                world_target_x = func_param.bl_way_points[num + 1][0], 
                world_target_y = func_param.bl_way_points[num + 1][1]
            )

            print("base_num " +str(base_num))
            print("r_bl_rot_to_target " +str(r_base_target))
            print("r_bl_rot_to_bl_tube " +str(r_near_target) +"\n")

            result=getInterSectionPoint.calLine(
                center_x=w_rob_bl_rot_x, 
                center_y=w_rob_bl_rot_y,
                p1_x = r_base_target[0],
                p1_y = r_base_target[1],
                p2_x = r_near_target[0],
                p2_y = r_near_target[1]
            )

            if result:
                print("result " +str(result))
                float(result[0].x)
                float(result[0].y)

        # result = getInterSectionTube.cal( 
        #     tube_rot_axis_x = w_rob_bl_rot_x,
        #     tube_rot_axis_y = w_rob_bl_rot_y,
        #     tube_x = w_bl_tube_x,
        #     tube_y = w_bl_tube_y,
        # )

        # if result:
        #     array = []
        #     map_intersec = Float32MultiArray(data=array)
        #     counter = 0

        #     print("publish " + str(math.degrees(radian)) + "\n")

        #     pub_intersec_pos.publish(map_intersec)

        pub_bl_tube_angle.publish(0.0) 
        #     # pub_br_tube_angle.publish(radian) 
                
    func_param.counter +=1
    rate.sleep()