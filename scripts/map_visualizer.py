#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def map_callback(float_array):
    map_data = Float32MultiArray(data=float_array)
    points_marker = Marker()
    points_marker.type = Marker.POINTS
    points_marker.id = 1
    points_marker.header.frame_id = "world"
    points_marker.header.stamp = rospy.Time.now()
    points_marker.ns = "maps"
    points_marker.action = Marker.ADD
    points_marker.pose.orientation.w=1.0
    points_marker.scale.x = 0.1
    points_marker.color.g = 1.0
    points_marker.color.a = 1.0

    for i in range(int(len(map_data.data.data)/2)):
        point = Point()
        point.x = float(map_data.data.data[i*2+0])
        point.y = float(map_data.data.data[i*2+1])
        point.z = float(0.0)
        points_marker.points.append(point)
    marker_pub.publish(points_marker)


rospy.init_node("map_visualizer")
marker_pub = rospy.Publisher("map_vis", Marker, queue_size=10)
canin_sub = rospy.Subscriber("map_data", Float32MultiArray, map_callback)
rospy.spin()
