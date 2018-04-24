#!/usr/bin/env python
#coding:utf-8
import rospy
from pid_ctrl.msg import flight_data
import numpy as np

def talker():
    pub  = rospy.Publisher('flight_data', flight_data, queue_size = 10)
    rospy.init_node('Publisher', anonymous = True)
    rate = rospy.Rate(20)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        t = rospy.Time.now() - start_time
        data = flight_data()
        data.t = t.to_sec()
        data.r = 0.
        data.pos = np.sin(data.t)
        data.pitch = np.cos(data.t) * 20
        pub.publish(data)
        print(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
            pass