#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import getWayPoints

array = []
map_data = Float32MultiArray(data=array)
bl_tube_data = Float32MultiArray(data=array)
br_tube_data = Float32MultiArray(data=array)


rospy.init_node("map_way_points_publisher")
map_pub = rospy.Publisher("map_data", Float32MultiArray, queue_size=10)
bl_tube_pub = rospy.Publisher("bl_tube_data", Float32MultiArray, queue_size=10)
br_tube_pub = rospy.Publisher("br_tube_data", Float32MultiArray, queue_size=10)

rate = rospy.Rate(100)

rob_way_points = getWayPoints.read_rob()
bl_tube_points = getWayPoints.read_bl_tube_points()
br_tube_points = getWayPoints.read_br_tube_points()

for i in range(len(rob_way_points)):
    map_data.data.append(rob_way_points[i][0])
    map_data.data.append(rob_way_points[i][1])

for i in range(len(bl_tube_points)):
    map_data.data.append(bl_tube_points[i][0])
    map_data.data.append(bl_tube_points[i][1])

for i in range(len(br_tube_points)):
    map_data.data.append(br_tube_points[i][0])
    map_data.data.append(br_tube_points[i][1])



while not rospy.is_shutdown():
    map_pub.publish(map_data)
    rate.sleep()