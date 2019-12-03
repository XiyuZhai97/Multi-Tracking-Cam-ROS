#!/usr/bin/env python
# Xiyu
import  sys
import  tty, termios
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
xyo_data = ""
FREQ = 1.0  # [Hz]
NOISY_OBSR = False
NOISY_CTRL = False
RENDER = False
# Road geometry
RADIUS = 1  # [m]
# Vehicle structure
WHEEL_TRACK = 0.12  # [m]
WHEEL_RADIUS = 0.0325  # [m]

# Dynamic constraints
V_MIN = 0  # [m/s]
V_MAX = 1.5  # [m/s]
W_MAX = np.pi  # [rad/s]
W_MIN = -np.pi  # [rad/s]
CONST_VEL = 0.25  # [m/s]

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global xyo_data
    xyo_data = data.data

def calc_vel_r(_):
    return CONST_VEL


def calc_yaw_r(_):
    return CONST_VEL / RADIUS


def calc_vel_ctrl(vel_r, x_err, y_err, c1):
    noisy = NOISY_OBSR
    if noisy:
        x_err = x_err * (1 + np.random.normal(0, 0.001))
        y_err = y_err * (1 + np.random.normal(0, 0.001))
    return vel_r + c1 * x_err / np.sqrt(1+x_err**2+y_err**2)


def calc_yaw_ctrl(vel_r, yaw_r, x_err, y_err, o_err, c2, c3):
    noisy = NOISY_OBSR
    if noisy:
        x_err = x_err * (1 + np.random.normal(0, 0.001))
        y_err = y_err * (1 + np.random.normal(0, 0.001))
        o_err = o_err * (1 + np.random.normal(0, 0.001))
    return \
        yaw_r + \
        c2 * vel_r * (y_err*np.cos(o_err/2)-x_err*np.sin(o_err/2)) / \
        np.sqrt(1+x_err**2+y_err**2) + \
        c3 * np.sin(o_err/2)


def calc_vel_left(vel, yaw, track, radius):
    return (2 * vel - yaw * track) / (2 * radius)


def calc_vel_right(vel, yaw, track, radius):
    return (2 * vel + yaw * track) / (2 * radius)

def step_unicycle_state(vel_left, vel_right, o, track, radius):
    noisy = NOISY_CTRL
    if noisy:
        vel_left = vel_left * np.random.choice([0.95, 1.0, 1.05])
        vel_right = vel_right * np.random.choice([0.95, 1.0, 1.05])
    dxdt = radius / 2 * (vel_left+vel_right) * np.cos(o)
    dydt = radius / 2 * (vel_left+vel_right) * np.sin(o)
    dodt = radius / track * (vel_right - vel_left)
    return dxdt, dydt, dodt


# Initial states
x_r0 = 0.0
y_r0 = 1.0
o_r0 = np.pi
x0 = 0.01
y0 = 1.01
o0 = 1.01*np.pi

# Controller parameters
dt = 1.0 / FREQ  # [s]
t0 = 0.0  # [s]
T = 30.0  # [s]
c1 = 2.0
c2 = 0.1
c3 = 1.0

# Start simulation
x_r = x_r0
y_r = y_r0
o_r = o_r0
x_r_list = []
y_r_list = []
o_r_list = []
vel_r_list = []
yaw_r_list = []
vel_left_r_list = []
vel_right_r_list = []
x = x0
y = y0
o = o0
x_list = []
y_list = []
o_list = []
vel_list = []
yaw_list = []
vel_left_list = []
vel_right_list = []

plt.rcParams['font.family'] = ['FreeSans', 'Arial']
render = RENDER
if render:
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    circle_x = [np.cos(angle) for angle in np.linspace(0, 2 * np.pi, 100)]
    circle_y = [np.sin(angle) for angle in np.linspace(0, 2 * np.pi, 100)]
    ax.plot(circle_x, circle_y, 'k')
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.axis('equal')
    scatter_handles = []

rospy.init_node('unicycle_control', anonymous=True)
pub = rospy.Publisher('axis', String, queue_size=10)
rate = rospy.Rate(FREQ) # 20hz

# i = 1
t = t0
# for temp in range(0,200):
    # fd = sys.stdin.fileno()
    # tty.setraw( fd )
    # ch = sys.stdin.read( 1 )

# for t in np.arange(t0, T, dt):
while t < T:
    t += dt
    x_r_list.append(x_r)
    y_r_list.append(y_r)
    o_r_list.append(o_r)
    x_list.append(x)
    y_list.append(y)
    o_list.append(o)

    vel_r = calc_vel_r(t)
    yaw_r = calc_yaw_r(t)
    x_err = np.cos(o) * (x_r-x) + np.sin(o) * (y_r-y)
    y_err = -np.sin(o) * (x_r-x) + np.cos(o) * (y_r-y)
    o_err = o_r - o
    vel = calc_vel_ctrl(vel_r, x_err, y_err, c1)
    yaw = calc_yaw_ctrl(vel_r, yaw_r, x_err, y_err, o_err, c2, c3)

    # TODO: Observe x, y, o from ROS subscriber
    # x = x + vel * np.cos(o) * dt
    # y = y + vel * np.sin(o) * dt
    # o = o + yaw * dt
    x_r = x_r + vel_r * np.cos(o_r) * dt
    y_r = y_r + vel_r * np.sin(o_r) * dt
    o_r = o_r + yaw_r * dt
    o_r = o_r%np.pi - np.pi/2

    rospy.Subscriber("axis_position", String, callback)
    xyo_data_split = xyo_data.split(',')
    try:
        i = int(xyo_data_split[0])
        x = float(xyo_data_split[1])
        y = float(xyo_data_split[2])
        o = float(xyo_data_split[3])
        print("11:"+str(x)+","+str(y)+","+str(o))
    except:
        print(t)
        t -= dt
        print(t)

        print("not enough data")
        continue

    # TODO: Actuate vel_left and vel_right through ROS publisher
    vel_left = calc_vel_left(vel, yaw, WHEEL_TRACK, WHEEL_RADIUS)/(2*np.pi)
    vel_right = calc_vel_right(vel, yaw, WHEEL_TRACK, WHEEL_RADIUS)/(2*np.pi)
    vel_left = round(abs(vel_left)/1.5 , 1)
    vel_right = round(abs(vel_right)/1.5 , 1)
    command = "11:l=" + str(vel_left) + ",r=" + str(vel_right)
    print(command)
    # if not rospy.is_shutdown():
    # if ch == 'q':
    #     print("Stop")
    #     pub.publish("1:l=0,r=0")
    #     break
    pub.publish(command)
    # vel_list.append(vel)
    # yaw_list.append(yaw)
    # vel_r_list.append(vel_r)
    # yaw_r_list.append(yaw_r)
    rate.sleep()



print('Done (:')
plt.rcParams['font.family'] = ['FreeSans', 'Arial']
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(x_r_list, y_r_list, 'b', label='Reference')
ax.plot(x_list, y_list, 'r', label='Trajectory')
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.legend()
ax.axis('equal')
plt.show()
