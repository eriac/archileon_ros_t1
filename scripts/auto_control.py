#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
import math
import time

x = Symbol("x")
y = Symbol("y")
world_target_position = [(0.5, 0.5), (0, 1.0), (-0.5, 1.5), (0, 2.0)]
move_speed = 0.1

class timer:
    def set_time(self, time):
        self.set = time
    def start_time(self, time):
        self.start = time

def callback(msg):
    if world_target_position == []:
        print("END OF POSITION")
    if (timer.set <= (time.time() - timer.start)):
        print("callback")
        print(timer.set -((time.time() + timer.start)))

        world_x = msg.data[0]
        world_y = msg.data[1]
        world_theta = msg.data[2]

        print("world_x")
        print(world_x)
        print("world_y")
        print(world_y)

        print("world_target_position x ")
        print(world_target_position[0][0])

        print("world_target_position y ")
        print(world_target_position[0][1])

        pr_x = world_target_position[0][0] - world_x
        pr_y = world_target_position[0][1] - world_y


        world_target_position.pop(0)
        pr = np.array([pr_x, pr_y])

        cos = np.cos(world_theta)
        sin = np.sin(world_theta)

        reverse_rotate = np.array([[cos, sin], [-sin, cos]])
        rob_target_position = np.dot(reverse_rotate, pr)
        x = rob_target_position[0]
        y = rob_target_position[1]

        print("x value is " + str(x))
        print("y value is " + str(y))


        # x**2 + (y - a)**2 = a**2
        a = (x**2 + y**2)/ (2 * y)
        move_curve = a

        print("y is " + str(y))
        print("a is " + str(a))

        if y == a:
            rad = math.pi / 2
        elif y < a:
            rad = math.atan2((a-y), x)
            # rad = math.atan2((a-y)/x)
        elif a < y:
            rad = math.atan2((y-a), x)

        print("RADIAN")
        print(math.degrees(rad))
        arc_circle = 2 * a * math.pi * (math.degrees(rad)/360)

        print("ARC CIRCLE")
        print(arc_circle)

        move_time = arc_circle / move_speed
        timer.set_time(move_time)
        print("SET TIME")
        print(timer.set)

        print("MOVE CURVE")
        print(move_curve)

        pub_curve.publish(1 / move_curve)
        timer.start_time(time.time())

        print("START TIME")
        print(timer.start)

        # print("x value is " + str(x))
        # print("y value is " + str(y))
        # print("theta value is " + str(world_theta))


    else:
        pass


timer = timer()
timer.set_time(0)
timer.start_time(0)

rospy.init_node("auto_control")
pub_speed = rospy.Publisher('move_speed', Float32, queue_size=1000)
pub_curve = rospy.Publisher('move_curve', Float32, queue_size=1000)


sub = rospy.Subscriber("robot_status", Float32MultiArray, callback)





rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub_speed.publish(move_speed)
    rate.sleep()
