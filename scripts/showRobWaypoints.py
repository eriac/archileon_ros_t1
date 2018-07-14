#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import getWayPoints
import getNearestPoint
import getErrorDist


class AreaMap():
    def __init__(self):
        self.main_points = getWayPoints.read_rob()
        self.now_target_num = 10

class func_world_rob_pos():
    x = 0 
    y = 0
    theta = 0       


def position_callback(msg):
    func_world_rob_pos.x = msg.data[0]
    func_world_rob_pos.y = msg.data[1]
    func_world_rob_pos.theta = msg.data[2]

    ########For Using Vive Tracker##########
    # func_world_rob_pos.theta = msg.data[2]  - (math.pi/4)* 3 


area_map = AreaMap()
rospy.init_node("show_error_dist")
pub_error = rospy.Publisher('dist_error', Float32, queue_size=1000)
sub_rob_status = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    base_point, forward_point = getNearestPoint.search_value(
        way_points=area_map.main_points,
        world_rob_x=func_world_rob_pos.x,
        world_rob_y=func_world_rob_pos.y,
    )
    L_rob_points = abs(getErrorDist.cal(
        point_1_x=base_point[0],point_1_y=base_point[1], 
        point_2_x=forward_point[0],point_2_y=forward_point[1], 
        world_rob_x=func_world_rob_pos.x, world_rob_y=func_world_rob_pos.y)
    )
    pub_error.publish(L_rob_points)
    print("L_rob_points " +str(L_rob_points))
    rate.sleep()