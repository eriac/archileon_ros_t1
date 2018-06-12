#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import read_way_points
import cal_move_curve
import judge_rob_is_goal


move_speed = 0.1
world_target_x = 1.0
world_target_y = 1.0

class cal_counter:
    def __init__(self):
        self.num = 0
    def add_count(self):
        self.num += 1
    def reset_num(self):
        self.num = 0

class now_move_curve:
    def set_move_curve(self, move_curve):
        self.move_curve = move_curve


def position_callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]

    if cal_counter.num == 0:
        print("/////////////////////////////////////////////////")
        print("First move curve")
        print(" ")
        move_curve = cal_move_curve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y)
        now_move_curve.set_move_curve(move_curve)
    # else:
    #     if judge_rob_is_goal.judge(world_rob_x, world_rob_y, world_target_x, world_target_y):
    #         print("/////////////////////////////////////////////////")
    #         print("Calculate the next goal point")
    #         print(" ")
    #         move_curve = cal_move_curve.cal(world_rob_x, world_rob_y, world_rob_theta)
    #         now_move_curve.set_move_curve(move_curve)
    #         cal_counter.add_count()
    #
    # if cal_counter.num == 0:
    #     cal_counter.add_count()

    pub_curve.publish(1.0 / now_move_curve.move_curve)
    pub_speed.publish(move_speed)

cal_counter = cal_counter()
now_move_curve = now_move_curve()
map_points=read_way_points.read_points()

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rospy.spin()
