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
import search_value
import cal_point_line

move_speed = 0.01
counter=0

class Counter():
    def __init__(self):
        self.num = 0

main_points=[[1.0,0.0], [1.5,0.5],[1.0, 1.0],[0, 1.0], [-0.5, 0.5],[0,0]]

world_target_x = None
world_target_y = None

def position_callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]

    if counter.num == 0:
        move_curve = cal_move_curve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=main_points[0][0], world_target_y=main_points[0][1])

        for i in range(3):
            pub_curve.publish(1.0 / move_curve)
            pub_speed.publish(move_speed)
        counter.num += 1
    else:
        print(world_rob_x)
        print(world_rob_y)

        base_num, next_num = search_value.getNearestPoint(map_points, world_rob_x, world_rob_y)
        print(base_num)
        if 0 < map_points[base_num][0] < 1.0 and map_points[base_num][1] == 0:
            world_target_x = main_points[0][0]
            world_target_y = main_points[0][1]

        elif 1.0 <= map_points[base_num][0] <= 1.5 and 0 < map_points[base_num][1] <= 0.5:
            world_target_x = main_points[1][0]
            world_target_y = main_points[1][1]

        elif 1.0 <= map_points[base_num][0] <= 1.5 and 0.5 < map_points[base_num][1] <= 1.0:
            world_target_x = main_points[2][0]
            world_target_y = main_points[2][1]

        elif 0 <= map_points[base_num][0] <= 1.0 and map_points[base_num][1] == 1.0:
            world_target_x = main_points[3][0]
            world_target_y = main_points[3][1]

        elif -0.5 <= map_points[base_num][0] < 0 and 0.5 < map_points[base_num][1] < 1.0:
            world_target_x = main_points[4][0]
            world_target_y = main_points[4][1]

        elif -0.5 <= map_points[base_num][0] <= 0 and 0 < map_points[base_num][1] <= 0.5:
            world_target_x = main_points[5][0]
            world_target_y = main_points[5][1]


        p1_x=map_points[base_num][0]
        p1_y=map_points[base_num][1]

        p2_x=map_points[next_num][0]
        p2_y=map_points[next_num][1]

        line_length = cal_point_line.getDistance(p1_x, p1_y, p2_x, p2_y, world_rob_x, world_rob_y)

        if line_length > 0.01:

            move_curve = cal_move_curve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y)
            for i in range(3):
                pub_curve.publish(1.0 / move_curve)
                pub_speed.publish(move_speed)

            while True:
                base_num, next_num = search_value.getNearestPoint(map_points, world_rob_x, world_rob_y)

                p1_x=map_points[base_num][0]
                p1_y=map_points[base_num][1]

                p2_x=map_points[next_num][0]
                p2_y=map_points[next_num][1]

                line_length = cal_point_line.getDistance(p1_x, p1_y, p2_x, p2_y, world_rob_x, world_rob_y)
                if line_length == 0:
                    break



counter = Counter()
map_points=read_way_points.read_points()


rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rospy.spin()
