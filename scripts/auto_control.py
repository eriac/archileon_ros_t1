#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
import math


# world_target_pos_list = [(0.5, 0.5), (0, 1.0), (-0.5, 1.5), (0, 2.0), (0.5, 2.5), (0, 3.0), (-0.5, 3.5), (0, 4.0)]
world_target_pos_list = [(0.5, 0.5), (0, 1.0), (-0.5, 1.5), (0, 2.0), (0.5, 1.5), (0, 1.0), (-0.5, 0.5), (0,0)]
move_speed = 0.1

class world_target_goal_point:
    def set_point(self, x, y):
        self.x = x
        self.y = y

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


def judge_rob_is_goal(temporal_world_rob_x, temporal_world_rob_y, world_goal_x, world_goal_y):
    result = False
    diff_x = abs(world_goal_x - temporal_world_rob_x)
    diff_y = abs(world_goal_y - temporal_world_rob_y)
    if cal_counter.num %2 == 1:
        if diff_y <= 0.005:
            result = True
    else:
        if diff_x <= 0.005:
            result = True
    return result

def cal_move_curve(world_rob_x, world_rob_y, world_rob_theta):
    print(world_target_pos_list)
    print(" ")
    print("world_rob_x " + str(world_rob_x))
    print("world_rob_y " + str(world_rob_y))
    print(" ")

    goal_x,goal_y = world_target_pos_list[0][0],world_target_pos_list[0][1]
    world_target_goal_point.set_point(goal_x, goal_y)
    print("world_target_goal_point.x " + str(world_target_goal_point.x))
    print("world_target_goal_point.y " + str(world_target_goal_point.y))
    print(" ")
    world_target_pos_list.pop(0)

    pr_x = world_target_goal_point.x - world_rob_x
    pr_y = world_target_goal_point.y - world_rob_y

    print("pr_x is " + str(pr_x))
    print("pr_y is " + str(pr_y))
    print(" ")

    pr = np.array([pr_x, pr_y])

    if world_rob_theta < 0:
        print("world_rob_theta is MINUS" + str(world_rob_theta))
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    if world_rob_theta >= 0:
        print("world_rob_theta is PLUS" + str(world_rob_theta))
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, sin], [-sin, cos]])


    rob_target_position = np.dot(rotate, pr)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]

    print("rob_target_x  is " + str(rob_target_x))
    print("rob_target_y  is " + str(rob_target_y))
    print(" ")

    radius = ((rob_target_x**2) + (rob_target_y**2) ) / (2 * rob_target_y)
    print("radius is " + str(radius))

    if rob_target_y == 0:
        rad == 0
    elif 0 < rob_target_y:
        rad = math.atan2(rob_target_x, (radius - rob_target_y))
    elif rob_target_y < 0:
        rad = math.atan2(rob_target_x, (rob_target_y - radius))

    move_curve = radius
    print("曲率" + str(1.0 / move_curve))
    print("/////////////////////////////////////////////////")
    return move_curve

def callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]


    if cal_counter.num == 0:
        print("/////////////////////////////////////////////////")
        print("First move curve")
        print(" ")
        move_curve = cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
        now_move_curve.set_move_curve(move_curve)
    else:
        if judge_rob_is_goal(world_rob_x, world_rob_y, world_target_goal_point.x, world_target_goal_point.y):
            print("/////////////////////////////////////////////////")
            print("Calculate the next goal point")
            print(" ")
            move_curve = cal_move_curve(world_rob_x, world_rob_y, world_rob_theta)
            now_move_curve.set_move_curve(move_curve)
            cal_counter.add_count()

    if cal_counter.num == 0:
        cal_counter.add_count()

    pub_curve.publish(1.0 / now_move_curve.move_curve)
    pub_speed.publish(move_speed)

cal_counter = cal_counter()
now_move_curve = now_move_curve()
world_target_goal_point = world_target_goal_point()

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)
rospy.spin()
