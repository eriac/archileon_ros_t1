#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import os
import sys


sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')


def br_tube_position_callback(float_msg):
    global world_br_tube_x
    global world_br_tube_y
    world_br_tube_x = float_msg.data[0]
    world_br_tube_y = float_msg.data[1]

    print(world_br_tube_x)
    print(world_br_tube_y)

    with open(os.path.dirname(
            os.path.abspath(__file__)) + "/../n_tube_way_points/test.txt", "a") as f:
        str_w_br_tube_x = str(world_br_tube_x)
        str_w_br_tube_y = str(world_br_tube_y)
        f.write("["+str_w_br_tube_x + "," + str_w_br_tube_y + "]" + "\n")


rospy.init_node("logger_br_tube")

sub_rob_status = rospy.Subscriber(
    "br_tube_status", Float32MultiArray, br_tube_position_callback)


# rate = rospy.Rate(100)

rospy.spin()
