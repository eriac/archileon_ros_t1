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

    print("Machine world_rob_x " + str(world_rob_x))
    print("Machine world_rob_y " + str(world_rob_y))
    print("Machine world_rob_theta " + str(world_rob_theta))
    #
    # # 今のロボの位置から、経路に対して最も近い点を２つ取ってくる
    # world_rob_base_point, world_rob_adj_point = getNearestPoint.search_value(rob_way_points, world_rob_x, world_rob_y)
    #
    # world_rob_base_point_x = world_rob_base_point[0]
    # world_rob_base_point_y = world_rob_base_point[1]
    # world_rob_adj_point_x = world_rob_adj_point[0]
    # world_rob_adj_point_y = world_rob_adj_point[1]
    #
    #
    # # do whileがないのでこう書く
    # #一番始めだけどこに向かっていくかをこう書かないと計算できない
    # if counter.num == 0:
    #     move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
    #     for i in range(3):
    #         pub_curve.publish(1.0 / move_curve)
    #         pub_speed.publish(move_speed)
    #     counter.num +=1
    #
    # else:
    #
    #     # ロボットが経路からどれだけ離れたかを計算
    #     L_rob_points = abs(getDist_rob_points.cal(point_1_x=world_rob_base_point_x,point_1_y=world_rob_base_point_y, point_2_x=world_rob_adj_point_x,point_2_y=world_rob_adj_point_y, world_rob_x=world_rob_x,world_rob_y=world_rob_y))
    #
    #     # print("L_rob_points " +str(L_rob_points))
    #     # ロボの位置から今のエリアを判断して、新しいエリアに入ったら曲率半径を計算する
    #     if nowArea.judge(world_rob_x, world_rob_y, now_target_num=area_map.now_target_num, way_points=rob_way_points) is True:
    #         area_map.now_target_num +=1
    #         print("ELSE")
    #         print("area_map.now_target_num " +str(area_map.now_target_num))
    #         move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
    #         for i in range(3):
    #             pub_curve.publish(1.0 / move_curve)
    #             pub_speed.publish(move_speed)
    #
    #
    #     if switch.L_rob_points_is_over_001 is False:
    #         # 経路からロボが1cm以上離れたら再度曲率半径を計算する
    #         if L_rob_points > 0.0050:
    #             move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
    #             if move_curve > min_curve:
    #                 for i in range(3):
    #                     pub_curve.publish(1.0 / move_curve)
    #                     pub_speed.publish(move_speed)
    #             # 計算し直した結果、曲率半径があまりにも小さすぎる=ゴールに近すぎる場合は
    #             # 次のゴールを新しいゴールに設定する
    #             else:
    #                 area_map.now_target_num +=1
    #                 move_curve = getMoveCurve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.main_points[area_map.now_target_num][0], world_target_y=area_map.main_points[area_map.now_target_num][1])
    #
    #                 for i in range(3):
    #                     pub_curve.publish(1.0 / move_curve)
    #                     pub_speed.publish(move_speed)
    #             switch.L_rob_points_is_over_001 = True
    #     if switch.L_rob_points_is_over_001 is True:
    #         if L_rob_points < error_threshold:
    #             switch.L_rob_points_is_over_001 = False



area_map = AreaMap()
counter = Counter()
switch = Switch()

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
pub_bl_tube_axis_angle = rospy.Publisher('bl_tube_axis_angle', Float32, queue_size=1000)
pub_br_tube_axis_angle = rospy.Publisher('br_tube_axis_angle', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status1", Float32MultiArray, position_callback)
sub = rospy.Subscriber("/vive/LHR_1CDCEA0B", Float32MultiArray, position_callback)

rospy.spin()
