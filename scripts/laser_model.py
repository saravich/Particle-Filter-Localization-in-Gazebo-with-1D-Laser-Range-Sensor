#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
import numpy as np
from statistics import mean
from statistics import variance
import seaborn as sns
import matplotlib.pyplot as plt
rospy.init_node('laser_data_capture')

r = rospy.Rate(0.005)

class sensor:

    def __init__(self, distance = .40, counter = 10000, sampling_time = 50):
        self.real_dists = []
        self.detected_dists = []
        self.sampling_time = sampling_time  # ms
        self.dist = distance
        self.counter = counter

    def calc_error_props(self):
        self.detected_dists = np.array(self.detected_dists)
        self.dist = np.array(self.dist)
        error = self.detected_dists - self.dist
        sensor_noise_mean = mean(error)
        sensor_noise_var = variance(error)
        print("Laser Sensor Mean: {}".format(sensor_noise_mean))
        print("Laser Sensor Variance: {}".format(sensor_noise_var))
        plt.figure;
        plt.hist(error, density=True, bins=20)
        plt.grid()
        plt.xlabel("Laser Range Error (m)")
        plt.ylabel("freq")
        plt.title("Sensor Error Distribution")
        plt.show()

    def callback(self, msg):
        if len(self.detected_dists) < self.counter:
            self.detected_dists.append(msg.range)
            self.real_dists.append(self.dist)
            print(len(self.detected_dists))
            print(msg.range)

    def listener(self):
        while len(self.detected_dists) < self.counter:
            rospy.init_node('laser_data_capture')
            sub = rospy.Subscriber('/vector/laser', Range, self.callback)
            rospy.sleep(0.02)

            if len(self.detected_dists) >= self.counter:
                break

        self.calc_error_props()


sensor_ = sensor()
sensor_.listener()