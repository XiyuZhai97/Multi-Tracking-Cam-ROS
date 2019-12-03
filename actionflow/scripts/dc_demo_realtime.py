#!/usr/bin/env python
# Xiyu
import  sys
import  tty, termios
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import math
plt.rcParams['font.family'] = ['Arial', 'FreeSans']
plt.rcParams['font.size'] = 16
# plt.style.use('dark_background')
xyo_data = ""

radius = 1  # [m]
circumference = 2 * np.pi * radius  # [m]
num_veh = 17
dt = 0.1  # [s]
buffer_length = 5  # [m]

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global xyo_data
    xyo_data = data.data
def plot(ax, positions_buffer, t):
    plt.cla()
    positions_array = np.array(positions_buffer)
    time_array = np.arange(t-buffer_length, t, dt)
    ax.plot(time_array, positions_array, '.b')
    ax.axvline(x=t, linewidth=3)
    ax.set_xlim([t-buffer_length, t+1])
    ax.set_ylim([0, circumference])
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Linear Displacement [m]')
    ax.text(x=t+0.1, y=circumference-1, s='present')
    plt.pause(0.001)


def main():
    rospy.init_node('dc_demo', anonymous=True)

    x_list = np.zeros(num_veh)
    y_list = np.zeros(num_veh)
    s_list = np.zeros(num_veh)
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(1, 1, 1)
    T = 10000  # [s]
    positions = np.arange(0, circumference, circumference/num_veh)
    positions_buffer = [positions for _ in range(int(buffer_length/dt))]
    speed = 0.1  # [m/s]
    # for t in np.arange(0, T, dt):
    t = 0
    while t < T:
        t += dt
        rospy.Subscriber("axis_position", String, callback)
        xyo_data_split = xyo_data.split(',')
        try:
            for car in range(num_veh):
                i = int(xyo_data_split[car * 3])
                x_list[car] = float(xyo_data_split[car * 3 + 1])
                y_list[car] = float(xyo_data_split[car * 3 + 2])
        except:
            t -= dt
            print("not enough data")
            continue
        
        for car in range(num_veh):
        
            s_list[car] = np.arctan2(abs(y_list[car]),x_list[car]) * radius
            if y_list[car] >= 0:
                pass
            else:
                s_list[car] = 2 * np.pi - s_list[car]

        plot(ax, positions_buffer, t)

        # positions = positions + np.array([
        #     np.random.normal(loc=speed, scale=0.01)*dt
        #     for _ in range(num_veh)])

        for car in range(num_veh):
            positions[car] = s_list[car] % circumference

        positions = positions % circumference # hack 
 
        positions_buffer.append(positions)
        positions_buffer.pop(0)


if __name__ == '__main__':
    main()
