#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import (String, Float64)
from geometry_msgs.msg import (Twist, TransformStamped)
from geometry_msgs.msg import Point
import tinyik
import numpy as np

EE_START = [-1.7, 0.0, 0.025]
EE_MID1 = [-0.2, 0.0, 1.8]
EE_MID2 = [0.0, 0.0, 2.0]
EE_MID3 = [0.2, 0.0, 1.8]
EE_END = [1.8, 0.0, 0.4]
DURATION = 5
HZ = 10
LOOP_RATE = 1/HZ
INIT_TIME = 3

def callback_input_cmd(data):
    global pub_cmd_joint1
    global pub_cmd_joint2
    global pub_cmd_joint3
    input = data.data
    print(input)
    if input == "init":
        init_pose(EE_START)

    elif input == "action":
    # for idx, val in enumerate(items):
        xyz_traj1 = cal_trajectory(EE_START, EE_MID1)
        xyz_traj2 = cal_trajectory(EE_MID1, EE_MID2)
        xyz_traj3 = cal_trajectory(EE_MID2, EE_MID3)
        xyz_traj4 = cal_trajectory(EE_MID3, EE_END)

        for i in range(len(xyz_traj1)):
            pub_cmd_joint1.publish(xyz_traj1[i][0])
            pub_cmd_joint2.publish(xyz_traj1[i][1])
            pub_cmd_joint3.publish(xyz_traj1[i][2])
            rospy.sleep(LOOP_RATE)
        rospy.sleep(1)


        for i in range(len(xyz_traj2)):
            pub_cmd_joint1.publish(xyz_traj2[i][0])
            pub_cmd_joint2.publish(xyz_traj2[i][1])
            pub_cmd_joint3.publish(xyz_traj2[i][2])
            rospy.sleep(LOOP_RATE)
        rospy.sleep(1)

        for i in range(len(xyz_traj3)):
            pub_cmd_joint1.publish(xyz_traj3[i][0])
            pub_cmd_joint2.publish(xyz_traj3[i][1])
            pub_cmd_joint3.publish(xyz_traj3[i][2])
            rospy.sleep(LOOP_RATE)

        rospy.sleep(1)

        for i in range(len(xyz_traj4)):
            pub_cmd_joint1.publish(xyz_traj4[i][0])
            pub_cmd_joint2.publish(xyz_traj4[i][1])
            pub_cmd_joint3.publish(xyz_traj4[i][2])

            rospy.sleep(LOOP_RATE)

def cal_trajectory(point1, point2):
    xyz_traj = []
    x_traj = np.linspace(point1[0], point2[0], num=DURATION * HZ)
    y_traj = np.linspace(point1[1], point2[1], num=DURATION * HZ)
    z_traj = np.linspace(point1[2], point2[2], num=DURATION * HZ)

    # for idx, val in enumerate(items):
    for i in range(len(x_traj)):
        angles = inverse_kinematics([x_traj[i], y_traj[i], z_traj[i]])
        xyz_traj.append(angles)

    return xyz_traj


def init_pose(point):
    pub_cmd_joint1.publish(0)
    rospy.sleep(1)
    pub_cmd_joint2.publish(0)
    rospy.sleep(1)
    pub_cmd_joint3.publish(0)
    rospy.sleep(1)

    angles = inverse_kinematics(point)
    pub_cmd_joint3.publish(angles[2])
    rospy.sleep(1)
    pub_cmd_joint2.publish(angles[1])
    rospy.sleep(1)
    pub_cmd_joint1.publish(angles[0])
    rospy.sleep(INIT_TIME)


def inverse_kinematics(end_effect=[0, 0]):

    # arm = tinyik.Actuator(['x', [0., 0., 4.], 'x', [0., 0., 2.], 'x', [0., 0., 2.]])
    arm2 = tinyik.Actuator(['y', [0., 0., 1.], 'y', [0., 0., 1.]])



    arm2.ee = end_effect
    angle1, angle2 = arm2.angles
    angle3 = 1.5708 - (angle1 + angle2)
    # if angle3 > 3.14:
    #     angle3 = angle3 - 3.14
    # elif angle3 < -3.14:
    #     angle3 = angle3 + 3.14
    # else:
    #     angle3 = angle3
    angle1 = round(angle1, 4)
    angle2 = round(angle2, 4)
    angle3 = round(angle3, 4)
    print(angle1, angle2, angle3)
    return [angle1, angle2, angle3]

    # arm.angles = [np.deg2rad([30, 60])]  #
    # print(arm.ee)
    # print(arm.angles)
    # print(arm.ee)
    #
    # arm.angles = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(90)]
    # print(arm.ee)
    #
    # arm.angles = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]
    # print(arm.ee)
    #
    # arm.ee = [0., -2., 6.]
    # print(arm.angles)


def rrbot_controller():
    # Init Node

    rospy.init_node('rrbot_controller', anonymous=None)

    global pub_cmd_joint1
    global pub_cmd_joint2
    global pub_cmd_joint3



    # rospy.Subscriber("cozmo_pose", Point, callbackFromSpeech)
    # rospy.Subscriber("rrbot_angle", BoundingBoxes, callbackFromObjRecog)

    rospy.Subscriber("input_cmd", String, callback_input_cmd)
    pub_cmd_joint1 = rospy.Publisher("rrbot/joint1_position_controller/command", Float64, queue_size=1)
    pub_cmd_joint2 = rospy.Publisher("rrbot/joint2_position_controller/command", Float64, queue_size=1)
    pub_cmd_joint3 = rospy.Publisher("rrbot/joint3_position_controller/command", Float64, queue_size=1)

    rospy.spin()



if __name__ == '__main__':
    rrbot_controller()
