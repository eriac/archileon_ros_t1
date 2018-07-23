#!/usr/bin/env python
# coding: UTF-8
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

import os
import sys
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')
import getWayPoints


def call_back(msg):
    global now_layer
    now_layer = msg.data
    br_tube_way_points = "nozzle_out_" + str(now_layer) + ".txt"
    br_way_points = getWayPoints.first_read(br_tube_way_points)
    for point in br_way_points:
        x = point[0]
        y = point[1]
        point_list.data.append(x)
        point_list.data.append(y)


rospy.init_node("br_waypoints_pub")
pub_br_way_points = rospy.Publisher(
    'br_way_points', Float32MultiArray, queue_size=1000)
sub_mode_change = rospy.Subscriber(
    "mode", Int32, call_back)

array = []
point_list = Float32MultiArray(data=array)
now_layer = 0

br_tube_way_points = "nozzle_out.txt"
br_way_points = getWayPoints.first_read(br_tube_way_points)
for point in br_way_points:
    x = point[0]
    y = point[1]
    point_list.data.append(x)
    point_list.data.append(y)


rate = rospy.Rate(10)
while not rospy.is_shutdown():

    pub_br_way_points.publish(point_list)

    rate.sleep()
