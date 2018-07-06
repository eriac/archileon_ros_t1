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

    w_bl_rot_pos, w_bl_pos, w_br_rot_pos, w_br_pos = getTubePosition.cal( 
    func_world_rob_pos.x, func_world_rob_pos.y, func_world_rob_pos.theta
    ) 
    func_world_bl_rot_pos.x = w_bl_rot_pos[0]
    func_world_bl_rot_pos.y = w_bl_rot_pos[1]

    func_world_bl_tube_pos.x = w_bl_pos[0]
    func_world_bl_tube_pos.y = w_bl_pos[1]

    for num in range(3):
        array = []
        bl_tube_pos = Float32MultiArray(data=array)
        bl_tube_pos.data.append(w_bl_pos[0])
        bl_tube_pos.data.append(w_bl_pos[1])
        pub_bl_tube_status.publish(bl_tube_pos)

    for num in range(3):
        array = []
        br_tube_pos = Float32MultiArray(data=array)
        br_tube_pos.data.append(w_br_pos[0])
        br_tube_pos.data.append(w_br_pos[1])
        pub_br_tube_status.publish(br_tube_pos)



area_map = AreaMap()
rospy.init_node("auto_control_tube")
pub_bl_tube_angle = rospy.Publisher('bl_rot_tube_angle', Float32, queue_size=1000)
pub_br_tube_angle = rospy.Publisher('br_rot_tube_angle', Float32, queue_size=1000)


pub_bl_tube_status = rospy.Publisher('bl_tube_status', Float32MultiArray, queue_size=1000)
pub_br_tube_status = rospy.Publisher('br_tube_status', Float32MultiArray, queue_size=1000)



sub_rob_status = rospy.Subscriber("robot_status", Float32MultiArray, position_callback)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    w_rob_bl_rot_x = func_world_bl_rot_pos.x
    w_rob_bl_rot_y = func_world_bl_rot_pos.y
    w_bl_tube_x = func_world_bl_tube_pos.x
    w_bl_tube_y = func_world_bl_tube_pos.y

    result = getInterSectionTube.cal( 
        tube_rot_axis_x = w_rob_bl_rot_x,
        tube_rot_axis_y = w_rob_bl_rot_y,
        tube_x = w_bl_tube_x,
        tube_y = w_bl_tube_y,
    )
    
    if result:
        for num in range(len(result)):
            radian=getTubeAngle.cal(
                origin_x=w_rob_bl_rot_x, origin_y=w_rob_bl_rot_y,
                u_x=w_bl_tube_x, u_y=w_bl_tube_y,
                v_x=float(result[num].x), v_y=float(result[num].y)
            )
            if abs(radian) < math.pi /2:
                print("degree " + str(math.degrees(radian)))

                pub_bl_tube_angle.publish(radian) 
                pub_br_tube_angle.publish(radian) 
                

    rate.sleep()