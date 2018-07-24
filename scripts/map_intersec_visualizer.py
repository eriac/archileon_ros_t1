#!/usr/bin/env python
# https://www.robotech-note.com/entry/2018/04/15/221524
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point


def position_callback(float_array):
    point = []
    point.append(float_array.data[0])
    point.append(float_array.data[1])
    global intersec_list
    intersec_list.append(point)


intersec_list = []

rospy.init_node("map_intersec_visualizer")
pub_intersec_pos = rospy.Publisher("map_intersec", Marker, queue_size=10)
sub_intersec_status = rospy.Subscriber(
    "bl_intersec_status", Float32MultiArray, position_callback)
sub_br_intersec_status = rospy.Subscriber(
    "br_intersec_status", Float32MultiArray, position_callback)


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
    marker_data.color.g = 1.0

    marker_data.scale.x = 0.003
    marker_data.scale.y = 0.003

    for point in intersec_list:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        marker_data.points.append(p)

    pub_intersec_pos.publish(marker_data)

    rate.sleep()
