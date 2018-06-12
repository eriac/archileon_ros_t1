#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import read_way_points

array = []
map_data = Float32MultiArray(data=array)

rospy.init_node("map_publisher")
map_pub = rospy.Publisher("map_data", Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)

map_points = read_way_points.read_points()

for i in range(len(map_points)):
    map_data.data.append(map_points[i][0])
    map_data.data.append(map_points[i][1])
map_pub.publish(map_data)

while not rospy.is_shutdown():
    map_pub.publish(map_data)
    rate.sleep()
