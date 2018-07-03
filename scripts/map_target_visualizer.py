#!/usr/bin/env python
# https://www.robotech-note.com/entry/2018/04/15/221524

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point




def position_callback(float_array):
    marker_data = Marker()
    marker_data.header.frame_id = "world"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.id = 0
    marker_data.ns = "map_target_visualizer"
    marker_data.action = Marker.ADD
    marker_data.pose.orientation.w=1.0
    marker_data.type = Marker.POINTS
    marker_data.color.r = 1.0
    marker_data.color.b = 1.0
    marker_data.color.a = 1.0
    marker_data.scale.x = 0.1
    marker_data.scale.y = 0.1
    p = Point()
    p.x = float_array.data[0]
    p.y = float_array.data[1]
    marker_data.points.append(p)
    for num in range(10):
        pub_map_target.publish(marker_data)



rospy.init_node("map_target_visualizer")
pub_map_target = rospy.Publisher("map_target", Marker, queue_size = 10)
sub_map_target = rospy.Subscriber("now_target_array", Float32MultiArray, position_callback)
rospy.spin()