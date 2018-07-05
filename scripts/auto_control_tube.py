#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sympy import *
import math
import getWayPoints
import getNearestPoint
import getDist_rob_points
import getTubePosition
import getTubeAngle
import getInterSectionTube
import getDistRobTarget


min_curve = 0.3
tube_radius = 0.105
error_threshold = 0.005
rob_way_points = getWayPoints.read_rob()

array = []
bl_tube_axis_angle_radian_array = Float32MultiArray(data=array)


class AreaMap():
    bl_points = getWayPoints.read_bl_tube_points()
    # br_points = getWayPoints.read_br_tube_points()

class func_world_rob_pos():
    x = 0 
    y = 0
    theta = 0


class func_world_bl_rot_pos():
    x = 0 
    y = 0

class func_world_bl_tube_pos():
    x = 0 
    y = 0




def position_callback(msg):
    func_world_rob_pos.x = msg.data[0]
    func_world_rob_pos.y = msg.data[1]
    func_world_rob_pos.theta = msg.data[2]

    w_bl_tube_rot_pos, w_bl_tube_pos, w_br_tube_rot_pos, w_br_tube_pos = getTubePosition.cal( 
    func_world_rob_pos.x, func_world_rob_pos.y, func_world_rob_pos.theta
    ) 
    func_world_bl_rot_pos.x = w_bl_tube_rot_pos[0]
    func_world_bl_rot_pos.y = w_bl_tube_rot_pos[1]

    func_world_bl_tube_pos.x = w_bl_tube_pos[0]
    func_world_bl_tube_pos.y = w_bl_tube_pos[1]


area_map = AreaMap()
rospy.init_node("auto_control")
pub_bl_tube_angle = rospy.Publisher('bl_tube_angle', Float32, queue_size=1000)
pub_br_tube_angle = rospy.Publisher('br_tube_angle', Float32MultiArray, queue_size=1000)
sub_rob_status = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    result = getInterSectionTube.cal( 
        tube_rot_axis_x = func_world_bl_rot_pos.x,
        tube_rot_axis_y = func_world_bl_rot_pos.y,
        tube_x = func_world_bl_tube_pos.x,
        tube_y = func_world_bl_tube_pos.y,
        way_points = AreaMap.bl_points
    )
    # print("result " + str(result))
    if result:
        for num in range(len(result)):
            radian=getTubeAngle.cal(
                origin_x=func_world_bl_rot_pos.x, origin_y=func_world_bl_rot_pos.y,
                u_x=func_world_bl_tube_pos.x, u_y=func_world_bl_tube_pos.y,
                v_x=float(result[num].x), v_y=float(result[num].y)
            )
            print(radian)
            if radian < math.pi /2:
                print("radian " + str(radian))
                pub_bl_tube_angle.publish(radian) 
    rate.sleep()