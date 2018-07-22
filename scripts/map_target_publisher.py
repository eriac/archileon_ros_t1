#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math

# import getDistRobTarget

import os
import sys
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')
import getMoveCurve
import getWayPoints


array = []
map_target = Float32MultiArray(data=array)


class AreaMap():
    def __init__(self):
        self.main_points = getWayPoints.read_rob()
        self.now_target_num = 35


class func_world_rob_pos():
    x = 0
    y = 0
    theta = 0


def position_callback(float_msg):
    func_world_rob_pos.x = float_msg.data[0]
    func_world_rob_pos.y = float_msg.data[1]
    func_world_rob_pos.theta = float_msg.data[2]

    # diff_rob_target = abs(getDistRobTarget.cal(
    #     func_world_rob_pos.x, func_world_rob_pos.y, func_world_rob_pos.theta,
    #     world_target_x=area_map.main_points[area_map.now_target_num][0],
    #     world_target_y=area_map.main_points[area_map.now_target_num][1]
    # ))

    diff_rob_target = (
        (area_map.main_points[area_map.now_target_num][0] -
         func_world_rob_pos.x)**2 +
        (area_map.main_points[area_map.now_target_num][1] -
         func_world_rob_pos.y)**2
    )

    if diff_rob_target < 0.03:
        area_map.now_target_num += 1


area_map = AreaMap()

rospy.init_node("map_target_pub")

pub_target = rospy.Publisher(
    'now_target_array', Float32MultiArray, queue_size=1000)

sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, position_callback)


rate = rospy.Rate(10)

while not rospy.is_shutdown():
    map_target.data.append(
        area_map.main_points[area_map.now_target_num][0])
    map_target.data.append(
        area_map.main_points[area_map.now_target_num][1])
    pub_target.publish(map_target)
    del map_target.data[:]

    rate.sleep()
