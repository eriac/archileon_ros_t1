#!/usr/bin/env python
#coding: UTF-8
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
import math
import time

x = Symbol("x")
y = Symbol("y")
world_target_position = [(1.0, 1.0), (0, 2.0), (-1, 3.0), (0, 4.0), (1, 5.0)]
move_speed = 0.1

class timer:
    def set_time(self, time):
        self.set = time
    def start_time(self, time):
        self.start = time

set_time = 0
start_time = 0

def callback(msg):
    # if world_target_position is []:
    #     print("END OF POSITION")

    if (timer.set is None and timer.start is None):

        print("//////////////////////////////////////////////////////")
        print("None callback")

        print(" ")
        world_rob_x = msg.data[0]
        world_rob_y = msg.data[1]
        world_rob_theta = msg.data[2]

        print("world_rob_x " + str(world_rob_x))
        print("world_rob_y " + str(world_rob_y))
        print(" ")

        print("world_target_position x " + str(world_target_position[0][0]))
        print("world_target_position y " + str(world_target_position[0][1]))
        print(" ")
        pr_x = world_target_position[0][0] - world_rob_x
        pr_y = world_target_position[0][1] - world_rob_y

        world_target_position.pop(0)

        pr = np.array([pr_x, pr_y])

        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)

        reverse_rotate = np.array([[cos, sin], [-sin, cos]])
        rob_target_position = np.dot(reverse_rotate, pr)
        rob_target_x = rob_target_position[0]
        rob_target_y = rob_target_position[1]

        print("rob_target_x  is " + str(rob_target_x))
        print("rob_target_y  is " + str(rob_target_y))
        print(" ")

        # x**2 + (y - a)**2 = a**2
        a = (rob_target_x**2 + rob_target_y**2)/ (2 * rob_target_y)
        print("a is " + str(a))
        pub_curve.publish(1.0 / a)

        move_curve = a
        # move_curve = 1


        print("a is " + str(a))
        print(" ")
        if rob_target_y == a:
            rad = math.pi / 2
        elif rob_target_y < a:
            rad = math.atan2((a - rob_target_y), rob_target_x)
            # rad = math.atan2((a-y)/x)
        elif a < rob_target_y:
            rad = math.atan2((rob_target_y - a), rob_target_x)

        print("角度 " +str(math.degrees(rad)))
        arc_circle = 2 * a * math.pi * (math.degrees(rad)/360)

        print("円弧の長さ " + str(arc_circle))
        move_time = arc_circle / move_speed
        timer.set_time(move_time)
        print("動いてほしい時間 " + str(move_time))

        print("曲率" + str(1 / move_curve))
        # pub_curve.publish(1 / move_curve)
        pub_curve.publish(1.0 / move_curve)
        timer.start_time(time.time())
        pub_curve.publish(1.0 / move_curve)
        pub_curve.publish(1.0 / move_curve)
        pub_curve.publish(1.0 / move_curve)


        print("START TIME " + str(timer.start))
        print("//////////////////////////////////////////////////////")

    else:
        if (timer.set <= (time.time() - timer.start)):
    # if (set_time <= (time.time() - start_time)):
            print("//////////////////////////////////////////////////////")
            print("Normal callback")
            print(timer.set -((time.time() + timer.start)))
            print(" ")
            world_rob_x = msg.data[0]
            world_rob_y = msg.data[1]
            world_rob_theta = msg.data[2]

            print("world_rob_x " + str(world_rob_x))
            print("world_rob_y " + str(world_rob_y))
            print(" ")

            print("world_target_position x " + str(world_target_position[0][0]))
            print("world_target_position y " + str(world_target_position[0][1]))
            print(" ")
            pr_x = world_target_position[0][0] - world_rob_x
            pr_y = world_target_position[0][1] - world_rob_y

            world_target_position.pop(0)

            pr = np.array([pr_x, pr_y])

            cos = np.cos(world_rob_theta)
            sin = np.sin(world_rob_theta)

            reverse_rotate = np.array([[cos, sin], [-sin, cos]])
            rob_target_position = np.dot(reverse_rotate, pr)
            rob_target_x = rob_target_position[0]
            rob_target_y = rob_target_position[1]

            print("rob_target_x  is " + str(rob_target_x))
            print("rob_target_y  is " + str(rob_target_y))
            print(" ")

            # x**2 + (y - a)**2 = a**2
            a = (rob_target_x**2 + rob_target_y**2)/ (2 * rob_target_y)

            pub_curve.publish(1.0 / a)
            move_curve = a
            # move_curve = 1
            print("a is " + str(a))
            print(" ")
            if rob_target_y == a:
                rad = math.pi / 2
            elif rob_target_y < a:
                rad = math.atan2((a - rob_target_y), rob_target_x)
                # rad = math.atan2((a-y)/x)
            elif a < rob_target_y:
                rad = math.atan2((rob_target_y - a), rob_target_x)

            print("角度 " +str(math.degrees(rad)))
            arc_circle = 2 * a * math.pi * (math.degrees(rad)/360)

            print("円弧の長さ " + str(arc_circle))
            move_time = arc_circle / move_speed
            timer.set_time(move_time)
            print("動いてほしい時間 " + str(move_time))

            print("曲率" + str(1 / move_curve))
            # pub_curve.publish(1 / move_curve)
            pub_curve.publish(1.0 / move_curve)
            timer.start_time(time.time())
            pub_curve.publish(1.0 / move_curve)
            pub_curve.publish(1.0 / move_curve)
            pub_curve.publish(1.0 / move_curve)


            print("START TIME " + str(timer.start))
            print("//////////////////////////////////////////////////////")
            # print("x value is " + str(x))
            # print("y value is " + str(y))
            # print("theta value is " + str(world_theta))





timer = timer()
timer.set_time(None)
timer.start_time(None)

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)


sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)





rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub_speed.publish(move_speed)
    rate.sleep()
