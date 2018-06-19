#!/usr/bin/python3

from random import *
import rospy


def add_list(behavior):
    dict_param = rospy.get_param('/behavior_rule')
    list_add_rule = dict_param[behavior]["add_list"]
    for add_rule in list_add_rule:
        rospy.set_param('/pre_condition/{}'.format(add_rule), 1)


def delete_list(behavior):
    dict_param = rospy.get_param('/behavior_rule')
    list_delete_rule = dict_param[behavior]["delete_list"]
    for delete_rule in list_delete_rule:
        rospy.set_param('/pre_condition/{}'.format(delete_rule), 0)


def cal_activation_energy(list_pre_condition):
    dict_param = rospy.get_param('/behavior_condition')
    list_behavior = list(dict_param.keys())
    list_energy = [0 for _ in range(len(list_behavior))]
    for behavior in list_behavior:
        energy = 0
        list_condition = dict_param[behavior]
        for pre_condition in list_pre_condition:
            if pre_condition in list_condition:
                if "_on_" in pre_condition:
                    energy += 1
                if "clear_" in pre_condition:
                    energy += 1

        # Adding Random Noise
        noise = uniform(-0.1, 0.1)
        energy += noise
        energy = round(energy, 4)
        list_energy[list_behavior.index(behavior)] = energy

    keys = list_behavior
    values = list_energy
    dict_energy = dict(zip(keys, values))
    print("Activation_energy :", dict_energy)

    return dict_energy


def selection_behavior(list_energy):
    behavior = max(list_energy, key=list_energy.get)
    return behavior


def do_behavior(pre_condition):
    dict_energy = cal_activation_energy(pre_condition)
    behavior = selection_behavior(dict_energy)
    return behavior


def check_param(prefix):
    dict_param = rospy.get_param(prefix)
    result = [name for name, value in dict_param.items() if value == 1]
    return result


def check_goal(pre_condition):
    dict_param = rospy.get_param('/goal')
    list_goal = list(dict_param.keys())
    set_goal = set(list_goal)
    set_pre_condition = set(pre_condition)
    intersection = set_goal & set_pre_condition
    complement = set_goal - intersection
    result = False
    print("Result :", list(intersection))
    if not list(complement):
        result = True
    return result
