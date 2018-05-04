#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
import math
import time


world_target_position = [(0.5, 0.5), (0, 1.0), (-0.5, 1.5), (0, 2.0), (0.5, 2.5)]
move_speed = 0.05

class timer:
    def set_time(self, time):
        self.set = time
    def start_time(self, time):
        self.start = time

def callback(msg):

    # print((timer.set -((time.time() - timer.start))))
    if (timer.set <= (time.time() - timer.start)):
        print("//////////////////////////////////////////////////////")
        print("Callback")

        print(" ")
        world_rob_x = msg.data[0]
        world_rob_y = msg.data[1]
        world_rob_theta = msg.data[2]

        print("world_rob_x " + str(world_rob_x))
        print("world_rob_y " + str(world_rob_y))
        print("world_rob_theta " + str(world_rob_theta))
        print(" ")

        print("world_target_position x " + str(world_target_position[0][0]))
        print("world_target_position y " + str(world_target_position[0][1]))
        print(" ")
        pr_x = world_target_position[0][0] - world_rob_x
        pr_y = world_target_position[0][1] - world_rob_y

        print("pr_x is " + str(pr_x))
        print("pr_y is " + str(pr_y))
        print(" ")

        print(world_target_position)
        world_target_position.pop(0)
        print(world_target_position)
        print(" ")

        pr = np.array([pr_x, pr_y])

        if world_rob_theta < 0:
            print("world_rob_theta is MINUS")
            print(world_rob_theta)
            print(" ")
            cos = np.cos(world_rob_theta)
            sin = np.sin(world_rob_theta)
            rotate = np.array([[cos, -sin], [sin, cos]])

        if world_rob_theta >= 0:
            print("world_rob_theta is PLUS")
            print(world_rob_theta)
            print(" ")
            cos = np.cos(world_rob_theta)
            sin = np.sin(world_rob_theta)
            rotate = np.array([[cos, sin], [-sin, cos]])


        rob_target_position = np.dot(rotate, pr)
        rob_target_x = rob_target_position[0]
        rob_target_y = rob_target_position[1]

        print("rob_target_x  is " + str(rob_target_x))
        print("rob_target_y  is " + str(rob_target_y))
        print(" ")

        radius = ((rob_target_x**2) + (rob_target_y**2) ) / (2 * rob_target_y)
        print("a is " + str(radius))
        print(" ")

        if rob_target_y == 0:
            rad == 0
        elif 0 < rob_target_y:
            rad = math.atan2(rob_target_x, (radius - rob_target_y))
            # rad = math.atan2((a-y)/x)
        elif rob_target_y < 0:
            rad = math.atan2(rob_target_x, (rob_target_y - radius))

        move_curve = radius
        print("曲率" + str(1.0 / move_curve))

        arc_circle = abs(2 * (move_curve) * math.pi * (math.degrees(rad)/360))
        print("円弧の長さ " + str(arc_circle))

        move_time = arc_circle / move_speed
        print("動いてほしい時間 " + str(move_time))

        timer.set_time(move_time)

        count = 0
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            pub_curve.publish(1.0 / move_curve)
                # start_time = time.time()
            count += 1
            if count == 3:
                timer.start_time(time.time() + 1)
                break
            rate.sleep()


timer = timer()
timer.set_time(0.0)
timer.start_time(0.0)

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)
sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)


rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub_speed.publish(move_speed)
    rate.sleep()
