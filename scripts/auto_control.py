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

move_speed = 0.01
min_curve = 0.3
tube_radius = 0.105
error_threshold = 0.005
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
        self.L_rob_points_is_over_001 = False

class AreaMap():
    def __init__(self):
        self.main_points = [[0.5, 0.0], [1.0,0.0], [1.5,0.5],[1.0, 1.0], [0.5, 1.0], [0, 1.0], [-0.5, 0.5],[0,0]]
        self.now_target_num = 0



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



    # 今のロボの位置から、経路に対して最も近い点を２つ取ってくる
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

        # ロボットが経路からどれだけ離れたかを計算
        L_rob_points = abs(getDist_rob_points.cal(point_1_x=rob_base_point_x,point_1_y=rob_base_point_y, point_2_x=rob_adj_point_x,point_2_y=rob_adj_point_y, world_rob_x=world_rob_x,world_rob_y=world_rob_y))

        print(L_rob_points)
        # ロボの位置から今のエリアを判断して、新しいエリアに入ったら曲率半径を計算する
        if nowArea.judge(world_rob_x, world_rob_y, now_target_num=area_map.now_target_num, way_points=rob_way_points) is True:
            area_map.now_target_num +=1
            print("ELSE")
            print("area_map.now_target_num " +str(area_map.now_target_num))
            move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
            for i in range(3):
                pub_curve.publish(1.0 / move_curve)
                pub_speed.publish(move_speed)


        if switch.L_rob_points_is_over_001 is False:
            # 経路からロボが1cm以上離れたら再度曲率半径を計算する
            if L_rob_points > 0.0050:
                move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
                if move_curve > min_curve:
                    for i in range(3):
                        pub_curve.publish(1.0 / move_curve)
                        pub_speed.publish(move_speed)
                # 計算し直した結果、曲率半径があまりにも小さすぎる=ゴールに近すぎる場合は
                # 次のゴールを新しいゴールに設定する
                else:
                    area_map.now_target_num +=1
                    move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])

                    for i in range(3):
                        pub_curve.publish(1.0 / move_curve)
                        pub_speed.publish(move_speed)
                switch.L_rob_points_is_over_001 = True
        if switch.L_rob_points_is_over_001 is True:
            if L_rob_points < error_threshold:
                switch.L_rob_points_is_over_001 = False



area_map = AreaMap()
counter = Counter()
switch = Switch()

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
pub_bl_tube_axis_angle = rospy.Publisher('bl_tube_axis_angle', Float32, queue_size=1000)
pub_br_tube_axis_angle = rospy.Publisher('br_tube_axis_angle', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rospy.spin()
