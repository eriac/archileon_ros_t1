#!/usr/bin/env python
# https://www.robotech-note.com/entry/2018/04/15/221524
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point


rob_way_points = []
bl_tube_way_points = []
br_tube_way_points = []


def rob_position_callback(float_array):
    point = []
    point.append(float_array.data[0])
    point.append(float_array.data[1])
    global rob_way_points
    rob_way_points.append(point)


def bl_tube_position_callback(float_array):
    point = []
    point.append(float_array.data[0])
    point.append(float_array.data[1])
    global bl_tube_way_points
    bl_tube_way_points.append(point)


def br_tube_position_callback(float_array):
    point = []
    point.append(float_array.data[0])
    point.append(float_array.data[1])
    global br_tube_way_points
    br_tube_way_points.append(point)


rospy.init_node("map_trajectory_visualizer")
pub_map_rob = rospy.Publisher("map_rob_trajectory", Marker, queue_size=10)

sub_rob_status = rospy.Subscriber(
    "robot_status", Float32MultiArray, rob_position_callback)

sub_bl_tube_status = rospy.Subscriber(
    "bl_tube_status", Float32MultiArray, bl_tube_position_callback)
sub_br_tube_status = rospy.Subscriber(
    "br_tube_status", Float32MultiArray, br_tube_position_callback)


rate = rospy.Rate(10)

while not rospy.is_shutdown():
    marker_data = Marker()
    marker_data.header.frame_id = "world"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.id = 3
    marker_data.action = Marker.ADD
    marker_data.pose.orientation.w = 1.0
    marker_data.type = Marker.POINTS
    marker_data.color.r = 1.0
    marker_data.color.a = 1.0
    marker_data.color.g = 0.5

    marker_data.scale.x = 0.01
    marker_data.scale.y = 0.01

    # for point in rob_way_points:
    #     p = Point()
    #     p.x = point[0]
    #     p.y = point[1]
    #     marker_data.points.append(p)

    for point in bl_tube_way_points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        marker_data.points.append(p)

    for point in br_tube_way_points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        marker_data.points.append(p)

    pub_map_rob.publish(marker_data)

    rate.sleep()
