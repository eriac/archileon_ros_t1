#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import getWayPoints
import getMoveCurve
import getDistRobTarget


array = []
map_target = Float32MultiArray(data=array)


class Counter():
    def __init__(self):
        self.num = 0


class AreaMap():
    def __init__(self):
        self.main_points = getWayPoints.read_rob()
        self.now_target_num = 35


class func_parameter():
    move_speed = 0.05
    move_curve = 0


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
        (area_map.main_points[area_map.now_target_num][0] - func_world_rob_pos.x)**2 +
        (area_map.main_points[area_map.now_target_num]
         [1] - func_world_rob_pos.y)**2
    )

    # print("target " +  str(area_map.now_target_num) + " [" +str(area_map.main_points[area_map.now_target_num][0])
    # + ",    " +str(area_map.main_points[area_map.now_target_num][1]) + "]")
    # print("machine " +   "[" + str(func_world_rob_pos.x) + ", " + str(func_world_rob_pos.y) + "]")
    # print("theta   " + str(func_world_rob_pos.theta))
    # print("move_curve " + str(func_parameter.move_curve) +"\n")

    if diff_rob_target < 0.03:
        area_map.now_target_num += 1


area_map = AreaMap()
counter = Counter()

rospy.init_node("autonomous_operation")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)

pub_target = rospy.Publisher(
    'now_target_array', Float32MultiArray, queue_size=1000)

sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, position_callback)


rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if counter.num == 0:
        move_curve = getMoveCurve.cal(world_rob_x=0, world_rob_y=0, world_rob_theta=0,
                                      world_target_x=area_map.main_points[area_map.now_target_num][0],
                                      world_target_y=area_map.main_points[area_map.now_target_num][1])
        func_parameter.move_curve = move_curve
        for num in range(10):
            pub_curve.publish(1.0 / func_parameter.move_curve)
            pub_speed.publish(func_parameter.move_speed)
        counter.num += 1
    else:
        move_curve = getMoveCurve.cal(func_world_rob_pos.x, func_world_rob_pos.y, func_world_rob_pos.theta,
                                      world_target_x=area_map.main_points[area_map.now_target_num][0],
                                      world_target_y=area_map.main_points[area_map.now_target_num][1])
        func_parameter.move_curve = move_curve
        pub_curve.publish(1.0 / func_parameter.move_curve)
        pub_speed.publish(func_parameter.move_speed)
        map_target.data.append(
            area_map.main_points[area_map.now_target_num][0])
        map_target.data.append(
            area_map.main_points[area_map.now_target_num][1])
        pub_target.publish(map_target)
        del map_target.data[:]

    rate.sleep()
