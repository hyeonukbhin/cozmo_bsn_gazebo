#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import (String, Float64)
import json
from time import sleep
from math import sqrt
import sys
from geometry_msgs.msg import (Twist, TransformStamped)
from darknet_ros_msgs.msg import BoundingBoxes
reload(sys)
sys.setdefaultencoding('utf-8')


global_obj_recog = ""

def callbackFromSpeech(data):
    human_speech = data.data
    print human_speech
    for i in range(0, 5):
        print "test"
    if human_speech:
        word_list = human_speech.split()

        print word_list
        if ('찾아' in word_list) and ('곰' in word_list):
            print "ok"
            # pub_cmd.publish("test")
            cmdLift(0.0)
            cmdLift(0.5)
            cmdLift(0.8)
            cmdLift(0.5)
            cmdLift(0.0)

            cmdHead(0.0)
            cmdHead(10.0)
            cmdHead(20.0)
            cmdHead(30.0)
            cmdHead(20.0)
            cmdHead(10.0)
            cmdTurn("left", 20)
            rospy.sleep(0.5)
            checkObject('bear')
            rospy.sleep(3)
            cmdTurn("left", 20)
            checkObject('bear')

        elif ('찾아' in word_list) and ('포켓몬' in word_list):
            print "ok"
            # cmdMove("front", 5)
            # cmdMove("front", 5)
            # rospy.sleep(1)
            # cmdTurn("left", 20)
            cmdSay("I will find a pokemon for you")

            cmdLift(0.0)
            cmdLift(0.5)
            cmdLift(0.8)
            cmdLift(0.5)
            cmdLift(0.0)

            cmdHead(0.0)
            cmdHead(10.0)
            cmdHead(20.0)
            cmdHead(30.0)
            cmdHead(20.0)
            cmdHead(10.0)
            rospy.sleep(3)
            cmdTurn("left", 25)
            rospy.sleep(0.5)

            checkObject('pokemon')
            rospy.sleep(3)
            cmdTurn("left", 25)
            rospy.sleep(0.5)

            checkObject('pokemon')
            rospy.sleep(3)
            # cmdSay("I will find a pokemon for you")



        else:
            print "Not understand"


def checkObject(obj_name):
    global global_obj_recog
    result = False
    print global_obj_recog
    if obj_name == global_obj_recog:
        rospy.sleep(1)
        cmdMove("front", 10)
        rospy.sleep(1)
        cmdSay("I found a %s" %obj_name)
        cmdLift(0.0)
        cmdLift(0.5)
        cmdLift(0.8)
        cmdLift(0.5)
        cmdLift(0.0)

        cmdHead(0.0)
        cmdHead(10.0)
        cmdHead(20.0)
        cmdHead(30.0)
        cmdHead(20.0)
        cmdHead(10.0)
        result = True


    return result

def cmdMove(direction, times):
    for i in range(0, times):
        # print "test"
        if direction is "front":
            x_val = 1
        elif direction is "back":
            x_val = -1

        twist = Twist()
        twist.linear.x = x_val
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


def callbackFromObjRecog(data):
    global global_obj_recog
    obj_recog = data.bounding_boxes
    obj_recog_class = obj_recog[0].Class
    obj_recog_prob = obj_recog[0].probability

    # print obj_recog_class
    # print type(obj_recog_class)

    # obj_recog_class = data.bounding_boxes.Class
    if obj_recog_class == 'pokemon' and obj_recog_prob > 0.2:
        # print "Found a pokemon!"
        global_obj_recog =obj_recog_class
        # cmdMove("front", 5)
        # rospy.sleep(1)
        # cmdSay("Found a pokemon!")
    elif obj_recog_class == 'bear' and obj_recog_prob > 0.2:
        global_obj_recog = obj_recog_class

        # print "Found a bear!"
        # cmdMove("front", 5)
        # rospy.sleep(1)
        # cmdSay("Found a bear!")
    else:
        global_obj_recog = ""

    # obj_recog_class == ""
    print global_obj_recog

def systemController():
    # Init Node

    rospy.init_node('system_controller', anonymous=None)
    # Init pub
    global pub_cmd_vel
    global pub_cmd_say
    global pub_cmd_lift
    global pub_cmd_head

    rospy.Subscriber("human_speech", String, callbackFromSpeech)
    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, callbackFromObjRecog)

    pub_cmd_vel = rospy.Publisher('key_teleop/cmd_vel', Twist, queue_size=1)
    pub_cmd_say = rospy.Publisher("cozmo/say", String, queue_size=1)
    pub_cmd_lift = rospy.Publisher("cozmo/lift_height", Float64, queue_size=1)
    pub_cmd_head = rospy.Publisher("cozmo/head_angle", Float64, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    systemController()
