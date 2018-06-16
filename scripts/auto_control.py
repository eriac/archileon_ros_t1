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

move_speed = 0.1
min_curve = 0.3
tube_radius = 0.105
rob_way_points = getWayPoints.read_rob()
bl_tube_tip_way_points = getWayPoints.read_bl_tube_points()
br_tube_tip_way_points = getWayPoints.read_br_tube_points()

array = []
tube_axis_angle_radian_array = Float32MultiArray(data=array)


class Counter():
    def __init__(self):
        self.num = 0
        self.global_num = 0
class Switch():
    def __init__(self):
        self.if_cal_move_curve_from_one_point_to_the_next_point = False

class AreaMap():
    def __init__(self):
        self.main_points = [[1.0,0.0], [1.5,0.5],[1.0, 1.0],[0, 1.0], [-0.5, 0.5],[0,0]]
        self.now_target_num = 0



def position_callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]


    # ロボの位置から相対的にノズルの位置を取ってくる
    rob_bl_tube_rot_axis_position, rob_bl_tube_tip_position,rob_br_tube_rot_axis_position,rob_br_tube_tip_position = getTubePosition.cal(world_rob_x, world_rob_y, world_rob_theta)

    # ノズルの回転の軸のworld座標系
    bl_tube_rot_axis_position_x = rob_bl_tube_rot_axis_position[0]
    bl_tube_rot_axis_position_y = rob_bl_tube_rot_axis_position[1]
    # ノズル本体のworld座標系
    bl_tube_tip_position_x = rob_bl_tube_tip_position[0]
    bl_tube_tip_position_y = rob_bl_tube_tip_position[1]

    #今のノズル本体の位置から、経路に対して最も近い点のインデックス２つを取ってくる
    bl_tube_base_point_num, bl_tube_adj_point_num = getNearestPoint.search_value(bl_tube_tip_way_points, bl_tube_tip_position_x, bl_tube_tip_position_y)

    #今のノズル本体の位置から、経路に対して最も近い２の点を取ってくる
    bl_tube_tip_base_point_x = bl_tube_tip_way_points[bl_tube_base_point_num][0]
    bl_tube_tip_base_point_y = bl_tube_tip_way_points[bl_tube_base_point_num][1]
    bl_tube_adj_point_x = bl_tube_tip_way_points[bl_tube_adj_point_num][0]
    bl_tube_adj_point_y = bl_tube_tip_way_points[bl_tube_adj_point_num][1]

    ans_1, ans_2=getTubeTipInterSection.cal(bl_tube_rot_axis_position_x, bl_tube_rot_axis_position_y, tube_radius, bl_tube_tip_base_point_x, bl_tube_tip_base_point_y, bl_tube_adj_point_x, bl_tube_adj_point_y)
    world_inter_section_x=ans_1[0][0]
    world_inter_section_y=ans_1[0][1]

    tube_axis_angle_radian=getTubeAngle.cal(bl_tube_rot_axis_position_x, bl_tube_rot_axis_position_y, bl_tube_tip_position_x, bl_tube_tip_position_y, world_inter_section_x, world_inter_section_y)

    print("tube_radian")
    print(math.degrees(tube_axis_angle_radian))

    # tube_axis_angle_radian_array.data.append(tube_axis_angle_radian)

    # # ノズルの回転の軸のworld座標系
    # br_tube_rot_axis_position_x = rob_br_tube_rot_axis_position[0]
    # br_tube_rot_axis_position_y = rob_br_tube_rot_axis_position[1]
    # # ノズル本体のworld座標系
    # br_tube_tip_position_x = rob_br_tube_tip_position[0]
    # br_tube_tip_position_y = rob_br_tube_tip_position[1]
    #
    # #今のノズル本体の位置から、経路に対して最も近い点のインデックス２つを取ってくる
    # br_tube_base_point_num, br_tube_adj_point_num = getNearestPoint.search_value(br_tube_tip_way_points, br_tube_tip_position_x, br_tube_tip_position_y)
    #
    # #今のノズル本体の位置から、経路に対して最も近い２の点を取ってくる
    # br_tube_tip_base_point_x = br_tube_tip_way_points[br_tube_base_point_num][0]
    # br_tube_tip_base_point_y = br_tube_tip_way_points[br_tube_base_point_num][1]
    # br_tube_adj_point_x = br_tube_tip_way_points[br_tube_adj_point_num][0]
    # br_tube_adj_point_y = br_tube_tip_way_points[br_tube_adj_point_num][1]
    #
    # ans_1, ans_2=getTubeTipInterSection.cal(br_tube_rot_axis_position_x, br_tube_rot_axis_position_y, tube_radius, br_tube_tip_base_point_x, br_tube_tip_base_point_y, br_tube_adj_point_x, br_tube_adj_point_y)
    # world_inter_section_x=ans_1[0][0]
    # world_inter_section_y=ans_1[0][1]
    #
    # tube_axis_angle_radian=getTubeAngle.cal(br_tube_rot_axis_position_x, br_tube_rot_axis_position_y, br_tube_tip_position_x, br_tube_tip_position_y, world_inter_section_x, world_inter_section_y)
    #
    # tube_axis_angle_radian_array.data.append(tube_axis_angle_radian)
    #
    # for num in range(3):
    #     pub_tube_axis_angle.publish(tube_axis_angle_radian_array)
    #

    #今のロボの位置から、経路に対して最も近い点を２つ取ってくる
    rob_base_point_num, rob_adj_point_num = getNearestPoint.search_value(rob_way_points, world_rob_x, world_rob_y)

    rob_base_point_x = rob_way_points[rob_base_point_num][0]
    rob_base_point_y = rob_way_points[rob_base_point_num][1]
    rob_adj_point_x = rob_way_points[rob_adj_point_num][0]
    rob_adj_point_y = rob_way_points[rob_adj_point_num][1]


    # do whileがないのでこう書く
    #一番始めだけどこに向かっていくかをこう書かないと計算できない
    if counter.num == 0:
        move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
        for i in range(3):
            pub_curve.publish(1.0 / move_curve)
            pub_speed.publish(move_speed)
        counter.num +=1


    else:
        # pass
        if nowArea.judge(world_rob_x, world_rob_y, now_target_num=area_map.now_target_num, way_points=rob_way_points) is True:
            area_map.now_target_num +=1
            print("ELSE")
            print("area_map.now_target_num " +str(area_map.now_target_num))
            move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
            for i in range(3):
                pub_curve.publish(1.0 / move_curve)
                pub_speed.publish(move_speed)



        # ロボットが経路からどれだけ離れたかを計算
        L_rob_points = abs(getDist_rob_points.cal(point_1_x=rob_base_point_x, point_1_y=rob_base_point_y, point_2_x=rob_adj_point_x, point_2_y=rob_adj_point_y, world_rob_x=world_rob_x, world_rob_y=world_rob_y))

        print(L_rob_points)
        if L_rob_points > 0.010:
            move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
            if move_curve > min_curve:
                for i in range(3):
                    pub_curve.publish(1.0 / move_curve)
                    pub_speed.publish(move_speed)

            else:
                area_map.now_target_num +=1
                move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])

                for i in range(3):
                    pub_curve.publish(1.0 / move_curve)
                    pub_speed.publish(move_speed)







area_map = AreaMap()
counter = Counter()
switch = Switch()

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
pub_tube_axis_angle = rospy.Publisher('tube_axis_angle', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rospy.spin()
