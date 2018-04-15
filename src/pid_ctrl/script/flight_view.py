#!/usr/bin/env python
#coding:utf-8
import rospy
from pid_ctrl.msg import flight_data
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anime


key = "s"
data_list = []
data_size_limit = 1000
startTime = rospy.Time
def get_flight_data(data): 
    global key
    t = rospy.Time.now() - startTime
    if key != "e" and len(data_list) < data_size_limit:
        print(t.secs)
        #rospy.loginfo(rospy.get_caller_id() + " : t = %f, r = %f, pos = %f, pitch = %f" , data.t, data.r, data.pos, data.pitch)
        data_list.append([data.t, data.r, data.pos, data.pitch])
    if len(data_list) == data_size_limit or len(data_list) > data_size_limit:
        key = "e"

def finish_callback(data):
    global key
    key = data.data

def listener():
    global startTime
    rospy.init_node("flight_view", anonymous = True)
    rospy.Subscriber("flight_data", flight_data, get_flight_data)
    rospy.Subscriber("finish", String, finish_callback)
    rate = rospy.Rate(100)
    print("ready")
    startTime = rospy.Time.now()
    while not rospy.is_shutdown():
        if key == "e":
            print("終わりだよ～")
            break
        rate.sleep()
        

if __name__ == '__main__':
    listener()
    print(data_list)
    print(len(data_list))