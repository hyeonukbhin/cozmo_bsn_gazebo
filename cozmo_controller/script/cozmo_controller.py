#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import (String, Float64)
import json
from time import sleep
from math import sqrt
import sys
from geometry_msgs.msg import (Twist, TransformStamped)
from geometry_msgs.msg import Point

from darknet_ros_msgs.msg import BoundingBoxes


global_obj_recog = ""

def callback_input_cmd(data):
    input = data.data
    print(input)
    if input == "go":
        cmdMove("front", 102)

        stop_move()
    if input == "back":
        cmdMove("back", 100)

        stop_move()


    # cmdTurn("left", 20)
    # stop_move()


def cmdMove(direction, times):
    for i in range(0, times):
        # print "test"
        if direction is "front":
            x_val = 2
        elif direction is "back":
            x_val = -2

        twist = Twist()
        twist.linear.x = x_val
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub_cmd_vel.publish(twist)
        rospy.sleep(0.05)


def stop_move():
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)
    rospy.sleep(0.1)


def cmdTurn(direction, times):

    for i in range(0, times):
        if direction is "left":
            z_val = 1.57
        elif direction is "right":
            z_val = -1.57

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = z_val

        pub_cmd_vel.publish(twist)
        rospy.sleep(0.1)

def cmdLift(degree):
    pub_cmd_lift.publish(degree)
    rospy.sleep(0.2)


def cmdSay(dialog):
    pub_cmd_say.publish(dialog)
    rospy.sleep(0.2)

def cmdHead(degree):
    pub_cmd_head.publish(degree)
    rospy.sleep(0.2)


def cozmo_controller():
    # Init Node

    rospy.init_node('cozmo_controller', anonymous=None)
    # Init pub
    global pub_cmd_vel
    global pub_cmd_say
    global pub_cmd_lift
    global pub_cmd_head


    rospy.Subscriber("input_cmd", String, callback_input_cmd)

    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_cmd_say = rospy.Publisher("cozmo/say", String, queue_size=1)
    pub_cmd_lift = rospy.Publisher("cozmo/lift_height", Float64, queue_size=1)
    pub_cmd_head = rospy.Publisher("cozmo/head_angle", Float64, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    cozmo_controller()
