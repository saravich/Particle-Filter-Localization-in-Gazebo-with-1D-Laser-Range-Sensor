#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion

# !/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import datetime
from statistics import mean
from statistics import variance
import numpy as np
from matplotlib.pyplot import plot
import matplotlib.pyplot as plt
from random import random

# import scipy.stats.norm.pdf as gaussian_dist

sensor_x = 0.0
sensor_y = 0.0

PI = 3.1415926535897

##-----------PID CONTROLLER DEFINITION--------------##
class PID:
    def __init__(self, Kp, Ki, Kd, setPoint=0, SampleTime=100):  # sample time is in millisconds
        if Kp < 0 or Kd < 0 or Ki < 0:
            print("invalid pid constants")
            return
        self.SampleTime = SampleTime
        sampleTimeInSec = SampleTime / 1000
        self.kp = Kp
        self.ki = Ki * sampleTimeInSec
        self.kd = Kd / sampleTimeInSec
        self.lastError = 0
        self.integralTerm = 0  # used for I term
        self.lastTime = datetime.datetime.now()
        #        startTime=startTime.seconds()*1000
        self.minOutput = 0
        self.maxOutput = 0
        self.Error = 0

    def Compute(self, feedback):
        presentTime = datetime.datetime.now()
        timeChange = (presentTime - self.lastTime).total_seconds() * 1000

        if timeChange > self.SampleTime:  # if a time interval equal to sample time has passed
            # Compute Working Error Variables
            self.Error = self.setPoint - feedback
            dError = self.Error - self.lastError  # error- last error
            self.integralTerm = self.integralTerm + self.ki * self.Error
            derivativeTerm = self.kd * dError
            proportionalTerm = self.kp * self.Error
            PIDOutput = self.integralTerm + derivativeTerm + proportionalTerm

            if self.maxOutput != 0 and PIDOutput > self.maxOutput:
                PIDOutput = self.maxOutput
            elif self.minOutput != 0 and PIDOutput < self.minOutput:
                PIDOutput = self.minOutput
            return PIDOutput
        self.lastTime = presentTime

    def SetSampleTime(self, newSampleTime):
        ratio = newSampleTime / self.SampleTime
        self.ki = self.ki * ratio
        self.kd = self.kd / ratio
        self.SampleTime = newSampleTime

    def SetOutputLimits(self, minOut, maxOut):
        self.minOutput = minOut
        self.maxOutput = maxOut

    def DefineSetpoint(self, coord):
        self.setPoint = coord

    def set_PID_constants(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd


pid_controller_x = PID(1.2, 0., 0.)
pid_controller_y = PID(0.5, 0., 0.)


def implement_PID(set_point_coord, feedback):
    # converting xyz coord to angle by inverse kinematics
    # system input is the return value of controller
    pid_controller_x.DefineSetpoint(set_point_coord[0])
    pid_controller_y.DefineSetpoint(set_point_coord[1])

    controllerOutput_x = pid_controller_x.Compute(feedback[0])
    controllerOutput_y = pid_controller_y.Compute(feedback[1])

    return [controllerOutput_x, controllerOutput_y]


##------------------------------------------------##
current_pose = [0, 0]
distance_threshold = 0.001
print(sensor_x)
def move(goal_point):
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    # Loop to move the turtle in an specified distance
    while (abs(current_pose[0] - goal_point[0]) > distance_threshold): #while not arrived at the specified x position
        speed = implement_PID(goal_point, current_pose)

        vel_msg.linear.x = speed[0]
        vel_msg.linear.y = speed[1]
        # print(speed[0])
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Takes actual time to velocity calculus
        t1 = rospy.Time.now().to_sec()
        # # Calculates distancePoseStamped
        # current_pose = speed*pid_controller.SampleTime*0.001 + prev_pose   # since PID sampleTime was defined in milliseconds

        #the current pose is obtained from odometry and is fed to the controller as a feedback
        current_pose[0] = sensor_x
        current_pose[1] = sensor_y

        # print('current pose: {}'.format(current_pose))

    # After the loop, stops the robot
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0

    # Force the robot to stop
    velocity_publisher.publish(vel_msg)


def read_odometry(msg):
    global sensor_x, sensor_y

    sensor_x = msg.pose.pose.position.x
    sensor_y = msg.pose.pose.position.y


rospy.init_node("MyTranslation")

sub = rospy.Subscriber("/odom", Odometry, read_odometry)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)

r = rospy.Rate(5)  # 5 Hz

data = []
noisy_data = []

dist = 0.5
for i in range(500):
    move([dist * (i+1), 0])

    noisy_data.extend([sensor_x])
    data.extend([0 + (i+1)*0.2])


noise = np.array(data) - np.array(noisy_data)

noise_miu = mean(noise)
noise_var = variance(noise)

print('miu=', noise_miu)
print('var=', noise_var)

plt.figure;
plt.hist(noise, density=True, bins=50)
plt.xlabel('Error (m)')
plt.xlabel('Error (m)')
plt.title("Error Distribution for {} cm distance with controller".format(dist * 100))
plt.grid()
plt.show()







