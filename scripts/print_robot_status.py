#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("print_robot_status")

class func_world_rob_pos():
    x = 0
    y = 0
    theta = 0

def position_callback(float_msg):
    func_world_rob_pos.x = float_msg.data[0]
    func_world_rob_pos.y = float_msg.data[1]
    func_world_rob_pos.theta = float_msg.data[2]

sub_rob_status = rospy.Subscriber(
    "/vive/robot_status", Float32MultiArray, position_callback)

rate = rospy.Rate(10)

while not rospy.is_shutdown():

    print("x " + str(func_world_rob_pos.x))
    print("y " + str(func_world_rob_pos.y))
    print("theta " + str(func_world_rob_pos.theta))
    
    rate.sleep()