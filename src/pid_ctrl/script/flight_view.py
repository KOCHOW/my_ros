#!/usr/bin/env python
#coding:utf-8
import rospy
from pid_ctrl.msg import flight_data
from std_msgs.msg import String

key = "s"

def get_flight_data(data):
    if key != "e":
        rospy.loginfo(rospy.get_caller_id() + " : t = %f, r = %f, pos = %f, pitch = %f" , data.t, data.r, data.pos, data.pitch)

def finish_callback(data):
    global key
    key = data.data

def listener():
    rospy.init_node("flight_view", anonymous = True)
    rospy.Subscriber("flight_data", flight_data, get_flight_data)
    rospy.Subscriber("finish", String, finish_callback)
    rate = rospy.Rate(100)
    print("ready")
    while not rospy.is_shutdown():
        if key == "e":
            break
        rate.sleep()
        

if __name__ == '__main__':
    listener()
    print("終わりだよ～")