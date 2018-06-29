#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import getWayPoints
from time import sleep
 
array = []
map_rob = Float32MultiArray(data=array)
 
class memoryTrack():
    def __init__(self):
        self.point_list =[]
 
def position_callback(msg):
    world_rob_x = msg.data[0]
    world_rob_y = msg.data[1]
    world_rob_theta = msg.data[2]
 
    memoryTrack.point_list.append(world_rob_x)
    memoryTrack.point_list.append(world_rob_y)
 
 
    for position in memoryTrack.point_list:
        map_rob.data.append(position)
        map_rob.data.append(position)
 
    for num in range(10):
        pub_track.publish(map_rob)
 
 
 
rospy.init_node("map_rob_publisher")
sub_robot_status = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
pub_track = rospy.Publisher("map_rob", Float32MultiArray, queue_size=10)
memoryTrack = memoryTrack()
rospy.spin()