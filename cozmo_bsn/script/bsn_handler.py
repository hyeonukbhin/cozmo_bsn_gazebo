#!/usr/bin/python3
import rospy
import bsn_core.functions as fc
from std_msgs.msg import String


def callback(data):
    print("callback test", data.data)


def main():
    global pub
    rospy.init_node("bsn_handler", anonymous=None)
    rospy.Subscriber("input", String, callback)
    pub = rospy.Publisher("output", String, queue_size=100)
    rate = rospy.Rate(1)
    print("==============================START==============================")
    while not rospy.is_shutdown():
        pre_condition = fc.check_param("/pre_condition")
        print("Pre_condition :", pre_condition)
        completion = fc.check_goal(pre_condition)
        print("Completion :", completion)
        if completion is False:
            behavior = fc.do_behavior(pre_condition)
            print("Behavior : \x1b[1;33m{}\x1b[1;m\n".format(behavior))
            # print("\n")
            fc.add_list(behavior)
            fc.delete_list(behavior)
        else:
            rospy.signal_shutdown("Goal Complete")
            print("===============================END===============================")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
