#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import read_way_points
import cal_move_curve
import cal_rob_target
import getNearestPoint
import getDist_rob_points
import tooCloseTargetPoint
import nextTarget

move_speed = 0.1
way_points = read_way_points.read()

class Counter():
    def __init__(self):
        self.num = 0
        self.global_num = 0
class Switch():
    def __init__(self):
        self.if_cal_move_curve_from_one_point_to_the_next_point = False

class AreaMap():
    def __init__(self):
        self.now_target_num = 0
        self.main_points = [[1.0,0.0], [1.5,0.5],[1.0, 1.0],[0, 1.0], [-0.5, 0.5],[0,0]]
        self.target_point = self.main_points[self.now_target_num]


def position_callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]
    world_bl_nozzle_x = msg.data[0] - 0.23
    world_bl_nozzle_y = msg.data[1] - 0.075
    world_br_nozzle_x = msg.data[0] - 0.23
    world_br_nozzle_y = msg.data[1] + 0.075

    base_point_num, adjacent_point_num = getNearestPoint.search_value(way_points, world_rob_x, world_rob_y)

    base_point_x = way_points[base_point_num][0]
    base_point_y = way_points[base_point_num][1]
    adjacent_point_x = way_points[adjacent_point_num][0]
    adjacent_point_y = way_points[adjacent_point_num][1]


    dist_points_to_rob = abs(getDist_rob_points.cal(point_1_x=base_point_x, point_1_y=base_point_y, point_2_x=adjacent_point_x, point_2_y=adjacent_point_y, world_rob_x=world_rob_x, world_rob_y=world_rob_y))

    if nextTarget.judge(world_rob_x, world_rob_y, now_target_num=area_map.now_target_num) is True:
        area_map.now_target_num += 1
        switch.if_cal_move_curve_from_one_point_to_the_next_point = False


    if dist_points_to_rob > 0.005:
        print("Error is over 0.005")
        # もし今のロボの場所がゴールまでとても近すぎた場合
        if tooCloseTargetPoint.judge(world_rob_x, world_rob_y, world_rob_theta, target_point=area_map.target_point) == True:
            area_map.now_target_num += 1
            print(area_map.now_target_num)
        switch.if_cal_move_curve_from_one_point_to_the_next_point = False

    if switch.if_cal_move_curve_from_one_point_to_the_next_point == False:
        print(area_map.now_target_num)
        print(area_map.target_point)
        move_curve = cal_move_curve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=area_map.target_point[0], world_target_y=area_map.target_point[1])
        for i in range(3):
            pub_curve.publish(1.0 / move_curve)
            pub_speed.publish(move_speed)

        switch.if_cal_move_curve_from_one_point_to_the_next_point = True




area_map = AreaMap()
counter = Counter()
switch = Switch()

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rospy.spin()
