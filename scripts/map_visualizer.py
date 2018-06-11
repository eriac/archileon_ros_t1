#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def map_callback(float_array):
    map_data = Float32MultiArray(data=float_array)
    line_strip = Marker()
    line_strip.header.frame_id = "world"
    line_strip.header.stamp = rospy.Time.now()
    line_strip.ns = "maps"
    line_strip.action = Marker.ADD
    line_strip.pose.orientation.w=1.0
    line_strip.id = 1
    line_strip.type = Marker.LINE_STRIP
    line_strip.scale.x = 0.1
    line_strip.color.b = 1.0
    line_strip.color.a = 1.0

    for i in range(int(len(map_data.data.data)/2)):
        point = Point()
        point.x = float(map_data.data.data[i*2+0])
        point.y = float(map_data.data.data[i*2+1])
        point.z = float(0.0)
        line_strip.points.append(point)
    marker_pub.publish(line_strip)


rospy.init_node("map_visualizer")
marker_pub = rospy.Publisher("map_vis", Marker, queue_size=10)
canin_sub = rospy.Subscriber("map_data", Float32MultiArray, map_callback)
rospy.spin()
