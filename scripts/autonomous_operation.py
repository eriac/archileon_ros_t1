#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
# import getMoveCurve
import os
import sys
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')
import getMoveCurve
import getWayPoints

move_speed = 0.1

world_rob_x = None
world_rob_y = None
world_rob_theta = None
world_target_x = None
world_target_y = None


def position_callback(float_msg):
    global world_rob_x
    global world_rob_y
    global world_rob_theta
    world_rob_x = float_msg.data[0]
    world_rob_y = float_msg.data[1]
    world_rob_theta = float_msg.data[2]


def target_callback(float_msg):
    global world_target_x
    global world_target_y
    world_target_x = float_msg.data[0]
    world_target_y = float_msg.data[1]


rospy.init_node("autonomous_operation")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub_target = rospy.Subscriber(
    'now_target_array',  Float32MultiArray, target_callback)
sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, position_callback)


rate = rospy.Rate(100)

while not rospy.is_shutdown():
    if world_rob_x is not None and world_rob_y is not None and world_rob_theta is not None and world_target_x is not None and world_target_y is not None:
        move_curve = getMoveCurve.cal(
            world_rob_x=world_rob_x,
            world_rob_y=world_rob_y,
            world_rob_theta=world_rob_theta,
            world_target_x=world_target_x,
            world_target_y=world_target_y
        )
        pub_curve.publish(1.0 / move_curve)
        pub_speed.publish(move_speed)

    rate.sleep()
