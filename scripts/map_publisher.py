#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

f = open("/mnt/hgfs/ROS_ENV/catkin_ws/src/archileon_ros_t1/scripts/way_points.txt", "r")
line = f.readline()

array = []
map_data = Float32MultiArray(data=array)

point_list=[]
while line:
    line = f.readline()
    point_list.append(line)

new_list = []
for i in range(1712):
    new_list.append(point_list[i].strip().strip(" ,"))

map_points=[]
for i in range(1712):
    xy_list =[]
    x = float(new_list[i][1:new_list[i].index(",")])*0.01
    y = float(new_list[0][new_list[0].index(",")+1: new_list[0].index("]")])*0.01
    xy_list.append(x)
    xy_list.append(y)
    map_points.append(xy_list)
f.close()


rospy.init_node("map_publisher")
map_pub = rospy.Publisher("map_data", Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    for i in range(1712):
        map_data.data.append(map_points[i][0])
        map_data.data.append(map_points[i][1])
    map_pub.publish(map_data)
    rate.sleep()
