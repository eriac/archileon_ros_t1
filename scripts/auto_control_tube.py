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
    # world座標系のロボの位置
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]

    print("Tube position " +str(msg))
    # print("Tube world_rob_x " + str(world_rob_x))
    # print("Tube world_rob_y " + str(world_rob_y))
    # ロボの位置から相対的にノズルの位置を取ってくる
    world_bl_tube_rot_axis_position, world_bl_tube_tip_position,world_br_tube_rot_axis_position,world_br_tube_tip_position = getTubePosition.cal(world_rob_x, world_rob_y, world_rob_theta)

    # print("////////////////////////////")
    # print("getTubePosition")
    # print("world_bl_tube_rot_axis_position " +str(world_bl_tube_rot_axis_position))
    # print("world_bl_tube_tip_position " +str(world_bl_tube_tip_position))
    # print("world_br_tube_rot_axis_position " +str(world_br_tube_rot_axis_position))
    # print("world_br_tube_tip_position " +str(world_br_tube_tip_position))
    # print("////////////////////////////")


    # BLのノズルの経路に対する角度を求めていく
    # world座標系のノズルの回転の軸の座標
    # ここを中心として円を描く
    world_bl_tube_rot_axis_position_x = world_bl_tube_rot_axis_position[0]
    world_bl_tube_rot_axis_position_y = world_bl_tube_rot_axis_position[1]
    # world座標系のノズルの先端の座標
    # ノズルの本体の位置から近くの点を探すため
    world_bl_tube_tip_position_x = world_bl_tube_tip_position[0]
    world_bl_tube_tip_position_y = world_bl_tube_tip_position[1]
    #今のノズル本体の位置から、経路に対して最も近い点２つを取ってくる
    world_bl_tube_tip_base_point, world_bl_tube_adj_point = getNearestPoint.search_value(bl_tube_tip_way_points, world_bl_tube_tip_position_x, world_bl_tube_tip_position_y)

    # print("////////////////////////////")
    # print("BL getNearestPoint")
    # print("world_bl_tube_tip_base_point " +str(world_bl_tube_tip_base_point))
    # print("world_bl_tube_adj_point " +str(world_bl_tube_adj_point))
    # print("////////////////////////////")


    # 何故かy座標にマイナスが必要
    world_bl_tube_tip_base_point_x = world_bl_tube_tip_base_point[0]
    world_bl_tube_tip_base_point_y = world_bl_tube_tip_base_point[1]
    world_bl_tube_adj_point_x = world_bl_tube_adj_point[0]
    world_bl_tube_adj_point_y = world_bl_tube_adj_point[1]


    try:
        result = getTubeTipInterSection.cal(center_x=world_bl_tube_rot_axis_position_x, center_y=world_bl_tube_rot_axis_position_y, radius=tube_radius, p1_x=world_bl_tube_tip_base_point_x, p1_y=world_bl_tube_tip_base_point_y, p2_x=world_bl_tube_adj_point_x, p2_y=world_bl_tube_adj_point_y)
        # print("////////////////////////////")
        # print("BL getTubeTipInterSection " +str(result))

        if result != []:
            inter_section_x = result[0].x
            inter_section_y = result[0].y

            tube_angle = getTubeAngle.cal(origin_x=world_bl_tube_rot_axis_position_x,origin_y=world_bl_tube_rot_axis_position_y, p1_x=world_bl_tube_tip_position_x, p1_y=world_bl_tube_tip_position_y, p2_x=inter_section_x, p2_y=inter_section_y)
            # print("////////////////////////////")
            # print("BL getTubeAngle " +str(math.degrees(tube_angle)))
            for num in range(5):
                pub_bl_tube_axis_angle.publish(tube_angle)
        else:
            pass
            # print("Fail BL getTubeAngle")
    except:
        pass
        # print("Fail BL getTubeTipInterSection")



    # BRのノズルの経路に対する角度を求めていく
    # world座標系のノズルの回転の軸の座標
    # ここを中心として円を描く
    world_br_tube_rot_axis_position_x = world_br_tube_rot_axis_position[0]
    world_br_tube_rot_axis_position_y = world_br_tube_rot_axis_position[1]
    # world座標系のノズルの先端の座標
    # ノズルの本体の位置から近くの点を探すため
    world_br_tube_tip_position_x = world_br_tube_tip_position[0]
    world_br_tube_tip_position_y = world_br_tube_tip_position[1]
    #今のノズル本体の位置から、経路に対して最も近い点２つを取ってくる
    world_br_tube_tip_base_point, world_br_tube_adj_point = getNearestPoint.search_value(br_tube_tip_way_points, world_br_tube_tip_position_x, world_br_tube_tip_position_y)

    # print("////////////////////////////")
    # print("BR getNearestPoint")
    # print("world_br_tube_tip_base_point " +str(world_br_tube_tip_base_point))
    # print("world_br_tube_adj_point " +str(world_br_tube_adj_point))
    # print("////////////////////////////")


    # 何故かy座標にマイナスが必要
    world_br_tube_tip_base_point_x = world_br_tube_tip_base_point[0]
    world_br_tube_tip_base_point_y = world_br_tube_tip_base_point[1]
    world_br_tube_adj_point_x = world_br_tube_adj_point[0]
    world_br_tube_adj_point_y = world_br_tube_adj_point[1]


    try:
        result = getTubeTipInterSection.cal(center_x=world_br_tube_rot_axis_position_x, center_y=world_br_tube_rot_axis_position_y, radius=tube_radius, p1_x=world_br_tube_tip_base_point_x, p1_y=world_br_tube_tip_base_point_y, p2_x=world_br_tube_adj_point_x, p2_y=world_br_tube_adj_point_y)
        # print("////////////////////////////")
        # print("BR getTubeTipInterSection " + str(result))

        if result != []:
            inter_section_x = result[0].x
            inter_section_y = result[0].y

            tube_angle = getTubeAngle.cal(origin_x=world_br_tube_rot_axis_position_x,origin_y=world_br_tube_rot_axis_position_y, p1_x=world_br_tube_tip_position_x, p1_y=world_br_tube_tip_position_y, p2_x=inter_section_x, p2_y=inter_section_y)
            print("////////////////////////////")
            print("BR getTubeAngle " +str(math.degrees(tube_angle)))
            for num in range(5):
                pub_br_tube_axis_angle.publish(tube_angle)
        else:
            pass
            # print("Fail BR getTubeAngle")
    except:
        pass
        # print("Fail BR getTubeTipInterSection")




rospy.init_node("auto_control_tube")
pub_bl_tube_axis_angle = rospy.Publisher('bl_tube_axis_angle', Float32, queue_size=1000)
pub_br_tube_axis_angle = rospy.Publisher('br_tube_axis_angle', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status2", Float32MultiArray, position_callback)
rospy.spin()
