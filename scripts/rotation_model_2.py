#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from random import random
from math import atan
import matplotlib.pyplot as plt
x = 0
y = 0
theta = 0.0
PI = 3.1415926535897


def rotate(speed, angle, clockwise):
    vel_msg = Twist()

    # Receiveing the user's input

    # Converting from angles to radians
    angular_speed = speed * 2 * PI / 360
    relative_angle = angle * 2 * PI / 360

    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = theta
    if current_angle < 0 :
        current_angle+= 360
    initial_angle = theta
    if initial_angle < 0 :
        current_angle+= 360
    while abs(current_angle - initial_angle) < relative_angle:
        # Checking if our movement is CW or CCW
        if clockwise == 1:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        print('while')
        print('current_angle:', current_angle * 180 / PI)
        print('relative_angle:', relative_angle * 180 / PI)
        print('vel', vel_msg.angular.z)
        velocity_publisher.publish(vel_msg)

        current_angle = theta

        if current_angle - initial_angle >= relative_angle:
            break
    # Forcing our robot to stop



def new_odometry(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    # theta = atan(rot_q.y / rot_q.x)
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # if theta < -0.1:
    #     theta += 2 * PI

rospy.init_node("MyRotation")

sub = rospy.Subscriber("/odom", Odometry, new_odometry)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)

r = rospy.Rate(5)  # 5 Hz

data = []
noisy_data = []
# import time
# time.sleep(2000000/1000000.0)
inc_angle = 5
initial_angle = theta

for i in range(30):
    init_point = theta
    data.extend([inc_angle + init_point])
    rotate(speed=200, angle=inc_angle, clockwise=0)
    noisy_data.extend([theta])

vel_msg = Twist()
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0
velocity_publisher.publish(vel_msg)

noise = np.array(data) - np.array(noisy_data)

noise_miu = np.mean(noise)
noise_var = np.var(noise)

print('miu=', noise_miu)
print('var=', noise_var)

# x = np.linspace(noise_miu - 3*noise_var, noise_miu + 3*noise_var, 100)
# # gaussian = gaussian_dist(x,noise_miu,noise_var)

plt.figure;
plt.hist(noise, density=True, bins=30)
plt.show()
