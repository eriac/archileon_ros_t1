#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import getWayPoints
import getMoveCurve
import cal_rob_target
import getNearestPoint
import getDist_rob_points
import tooCloseTargetPoint
import getTubePosition
import nowArea
import getTubeAngle
import getTubeTipInterSection


tube_radius = 0.105

bl_tube_tip_way_points = getWayPoints.read_bl_tube_points()
br_tube_tip_way_points = getWayPoints.read_br_tube_points()

array = []
tube_axis_angle_radian_array = Float32MultiArray(data=array)

def position_callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]


    # ロボの位置から相対的にノズルの位置を取ってくる
    world_rob_bl_tube_rot_axis_position, world_rob_bl_tube_tip_position,world_br_tube_rot_axis_position,world_br_tube_tip_position = getTubePosition.cal(world_rob_x, world_rob_y, world_rob_theta)


    # world座標系のノズルの回転の軸の座標
    # ここを中心として円を描く
    world_rob_bl_tube_rot_axis_position_x = world_rob_bl_tube_rot_axis_position[0]
    world_rob_bl_tube_rot_axis_position_y = world_rob_bl_tube_rot_axis_position[1]

    # world座標系のノズルの本体の座標
    # ノズルの本体の位置から近くの点を探すため
    world_rob_bl_tube_tip_position_x = world_rob_bl_tube_tip_position[0]
    world_rob_bl_tube_tip_position_y = world_rob_bl_tube_tip_position[1]

    #今のノズル本体の位置から、経路に対して最も近い点のインデックス２つを取ってくる
    bl_tube_tip_base_point_num, bl_tube_tip_adj_point_num = getNearestPoint.search_value(bl_tube_tip_way_points, world_rob_bl_tube_tip_position_x, world_rob_bl_tube_tip_position_y)

    #今のノズル本体の位置から、経路に対して最も近い２の点を取ってくる
    # 何故かy座標にマイナスが必要
    world_bl_tube_tip_base_point_x = bl_tube_tip_way_points[bl_tube_tip_base_point_num][0]
    world_bl_tube_tip_base_point_y = -bl_tube_tip_way_points[bl_tube_tip_base_point_num][1]
    world_bl_tube_adj_point_x = bl_tube_tip_way_points[bl_tube_tip_adj_point_num][0]
    world_bl_tube_adj_point_y = -bl_tube_tip_way_points[bl_tube_tip_adj_point_num][1]

    print("world_rob_bl_tube_rot_axis_position")
    print(world_rob_bl_tube_rot_axis_position_x)
    print(world_rob_bl_tube_rot_axis_position_y)
    print()
    print("tube_radius")
    print(tube_radius)
    print()
    print("world_bl_tube_tip_base_point")
    print(world_bl_tube_tip_base_point_x)
    print(world_bl_tube_tip_base_point_y)
    print()
    print("world_bl_tube_adj_point")
    print(world_bl_tube_adj_point_x)
    print(world_bl_tube_adj_point_y)

    try:
        result = getTubeTipInterSection.cal(center_x=world_rob_bl_tube_rot_axis_position_x, center_y=world_rob_bl_tube_rot_axis_position_y, radius=tube_radius, p1_x=world_bl_tube_tip_base_point_x, p1_y=world_bl_tube_tip_base_point_y, p2_x=world_bl_tube_adj_point_x, p2_y=world_bl_tube_adj_point_y)
        print(result)

        if result != []:
            inter_section_x = result[0].x
            inter_section_y = result[0].y

        tube_angle = getTubeAngle.cal(origin_x=world_rob_bl_tube_rot_axis_position_x,origin_y=world_rob_bl_tube_rot_axis_position_y, p1_x=world_rob_bl_tube_tip_position_x, p1_y=world_rob_bl_tube_tip_position_y, p2_x=inter_section_x, p2_y=inter_section_y)

        print("tube_angle")
        print(math.degrees(tube_angle))
        pub_bl_tube_axis_angle.publish(tube_angle)

    except:
        print("失敗")


pub_tube_axis_angle=[]

rospy.init_node("auto_control_tube")
pub_bl_tube_axis_angle = rospy.Publisher('bl_tube_axis_angle', Float32, queue_size=1000)
pub_br_tube_axis_angle = rospy.Publisher('br_tube_axis_angle', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rospy.spin()
