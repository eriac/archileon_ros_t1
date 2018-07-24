#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import os
import sys
sys.path.append(os.path.dirname(
    os.path.abspath(__file__)) + '/../my_ros_utils')
import getWayPoints


way_points = []

rospy.init_node("map_way_points_publisher")
pub_map_way_points = rospy.Publisher("map_way_points", Marker, queue_size=10)


r_way_points = "robot.txt"
bl_tube_way_points = "nozzle_in.txt"
br_tube_way_points = "nozzle_out.txt"
rob_way_points = getWayPoints.first_read(r_way_points)
bl_tube_points = getWayPoints.first_read(bl_tube_way_points)
br_tube_points = getWayPoints.first_read(br_tube_way_points)


for i in range(len(rob_way_points)):
    way_points.append(rob_way_points[i][0])
    way_points.append(rob_way_points[i][1])

for i in range(len(bl_tube_points)):
    way_points.append(bl_tube_points[i][0])
    way_points.append(bl_tube_points[i][1])

for i in range(len(br_tube_points)):
    way_points.append(br_tube_points[i][0])
    way_points.append(br_tube_points[i][1])

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    marker_data = Marker()
    marker_data.header.frame_id = "world"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.id = 1
    marker_data.action = Marker.ADD
    marker_data.pose.orientation.w = 1.0
    marker_data.type = Marker.POINTS
    marker_data.color.g = 1.0
    marker_data.color.a = 1.0
    # marker_data.lifetime = rospy.Duration(1)

    marker_data.scale.x = 0.01
    marker_data.scale.y = 0.01

    # print(way_points)
    for point in range(0, len(way_points), 2):
        p = Point()

        p.x = way_points[point]
        p.y = way_points[point + 1]
        marker_data.points.append(p)

    pub_map_way_points.publish(marker_data)

    rate.sleep()
