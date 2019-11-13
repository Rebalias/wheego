#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Path
import math
import numpy as np
from rospy.numpy_msg import numpy_msg
import matplotlib.pyplot as plt
import Path_Follow_Calls as carpull


x_val_array = np.array([])
y_val_array = np.array([])
init_pose = 0
lookahead_max = 3
simulation =False
k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 2.0  # speed proportional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle
max_ldist = 3

old_nearest_point_index = None
show_animation = False
show_animation2 =True



def check_pose(msg):
    global init_pose
    current = msg.pose.orientation.z
    init_pose = current


def trace_array(msg):
    #global pub_array
    global x_val_array
    global y_val_array
    global init_pose
    hold = msg.poses
    carpull.reset_dist(None)
    #make sure its not putting out no arrays or error reports
    if len(hold) > 1:
        #clear old array values if new ones need to be sent
        if len(x_val_array) >= 2:
            # print(len(x_val_array))
            x_val_array = np.array([])
            y_val_array = np.array([])
        #index poses into numpy arrays
        for i in range(len(hold)):
            list_vals = hold[i]
            list_x_vals = list_vals.pose.position.x
            list_y_vals = list_vals.pose.position.y
            x_val_array = np.append(x_val_array, [list_x_vals])
            y_val_array = np.append(y_val_array, [list_y_vals])
        cx = x_val_array[::-1]
        cy= y_val_array[::-1]
        target_speed = 10.0 / 6.3
        T = 4.0
        #get car state from car_pull state class
        state = carpull.State(x=cx[0], y=cy[0], yaw=init_pose, v=0.0)
        lastIndex = len(cx) - 1
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        target_ind = carpull.calc_target_index(state, cx, cy)

        while T >= time and lastIndex > target_ind:
            ai = carpull.PIDControl(target_speed, state.v)
            di, target_ind = carpull.pure_pursuit_control(state, cx, cy, target_ind)
            state = carpull.update(state, ai, di)

            time = time + dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            steer_a.publish(di)
            if simulation is True:  # pragma: no cover
                plt.cla()
                carpull.plot_arrow(state.x, state.y, state.yaw)

                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

    #car_handling()




rospy.init_node('path_array', anonymous=True)
r = rospy.Rate(1)
steer_a = rospy.Publisher('steer_angle', Float32, queue_size = 1)
rospy.Subscriber('/car_traject',PoseStamped, check_pose)
rospy.Subscriber('/sPath',Path, trace_array)
while not rospy.is_shutdown():


    #a = np.array(send, dtype=np.float32)
    #pub.publish(a)
    r.sleep()

