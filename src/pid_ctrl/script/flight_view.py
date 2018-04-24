#!/usr/bin/env python
#coding:utf-8
import rospy
from pid_ctrl.msg import flight_data
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anime


round = lambda x: (x * 2 + 1)//2

key = "s"
data_list = []
data_size_limit = 200
drone_len = 0.3
startTime = rospy.Time
pos = []
t = []

def get_flight_data(data): 
    global key
    if key != "e" and len(data_list) < data_size_limit:
        print(data.t)
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
    rate = rospy.Rate(200)
    print("ready")
    startTime = rospy.Time.now()
    while not rospy.is_shutdown():
        if key == "e":
            break
        rate.sleep()
        
def update(x,ims,ax):
    xi = []
    eta = []
    pos.append(x[2])
    t.append(x[0])

    theta = np.deg2rad(x[3])
    xi.append(x[2] - drone_len * np.cos(theta))
    xi.append(x[2] + drone_len * np.cos(theta))
    eta.append(drone_len * np.sin(theta))
    eta.append(- drone_len * np.sin(theta))
    drone_center, = [ax.scatter(x[2], 0, c = "b")]
    drone_body, = ax.plot(xi,eta, c = "r")
    time, = [ax.annotate("t = " + str(round(x[0] * 10.) / 10.) + "[s]",
                        xy = (3, 2),
                        horizontalalignment = 'right',
                        verticalalignment = 'top')]
    t_pos, = ax2.plot(t, pos, c = "r")

    ims.append([drone_body, drone_center, time, t_pos])

if __name__ == '__main__':
    fig = plt.figure()
    listener()
    ims = []
    data = np.array(data_list)
    ax = plt.subplot(221)
    ax2 = plt.subplot(223)
    ax2.plot(data[:,0], data[:,1])
    #ax2.plot(data[:,0], data[:,2])
    ax.set_xlim(-1, 3)
    ax.set_ylim(-2, 2)
    for x in data_list:
        update(x, ims, ax)
    data_delta = np.diff(data_list, axis = 0)
    delta_t = int(np.mean(data_delta, axis = 0)[0] * 1000)
    print("delta_t = " + str(delta_t))
    ani = anime.ArtistAnimation(fig, ims, interval = delta_t)
    ani.save("hoge.mp4", writer = "ffmpeg")

    print("終わりだよ～")
    plt.show()
        

