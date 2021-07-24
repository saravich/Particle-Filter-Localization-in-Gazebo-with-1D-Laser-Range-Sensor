#! /usr/bin/python
from os.path import expanduser
import xml.etree.ElementTree as ET
import shapely
from shapely.geometry import LineString, Point, Polygon
from math import cos
from math import sin
import matplotlib.pyplot as plt
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import pi
from sensor_msgs.msg import Range
import scipy
from numpy.random import uniform
from math import atan
from math import sqrt
import scipy.stats
from random import random
from math import *
import time
home = expanduser("~")
map_address = home + '/catkin_ws/src/anki_description/world/exam.world'


# map_address = '../worlds/sample1.world'


############################################## Particle Filter #########################################################
class ParticleFilter():

    def __init__(self, map_, translation_model, rotation_model, sensor_model, num_of_particles=100, window_width=800,
                 window_height=600, name_of_window='Particle Filter'):

        self.map_ = map_
        self.num_of_particles = num_of_particles
        self.num_of_parameters_of_each_particle = 6
        self.translation_model = translation_model
        self.rotation_model = rotation_model
        self.sensor_model = sensor_model
        self.window_width = window_width
        self.window_height = window_height

        # Read map ranges for particles initialization
        self.map_x_range = self.map_.X_range
        self.map_y_range = self.map_.Y_range

        # Do algorithm initializations
        self.particles = np.empty((self.num_of_particles, 4))
        self.particles_laser_intersect = []
        self.particles_laser_range = []
        self.create_uniform_particles()
        self.init_weights()
        self.robot_laser_val = 0.4
        #variable to monitor convergence percentage of the particles
        self.percent_conv = 0

    def create_uniform_particles(self):
        self.particles[:, 0] = uniform(self.map_x_range[0], self.map_x_range[1], size=self.num_of_particles)
        self.particles[:, 1] = uniform(self.map_y_range[0], self.map_y_range[1], size=self.num_of_particles)
        self.particles[:, 2] = np.random.choice([-pi / 2, pi / 2, pi, 0], size=self.num_of_particles)

    def init_weights(self):
        self.weights = np.array([1.0] * self.num_of_particles)
        self.weights.fill(1. / self.num_of_particles)

    def predict(self, d, dtheta, yaw_setpoint):
        # predict motion of each particle based on the motion model driven from phase 1
        # update the particles first, second and third elements corresponding to their x, y, and heading angle

        self.particles[:, 0] += (d * np.cos(yaw_setpoint) + np.random.normal(self.translation_model[0], self.translation_model[1],
                                                      self.num_of_particles))
        self.particles[:, 1] += (d * np.sin(yaw_setpoint)+ np.random.normal(self.translation_model[0], self.translation_model[1],
                                                      self.num_of_particles))
        self.particles[:, 2] += dtheta + np.random.normal(self.rotation_model[0], self.rotation_model[1],
                                                          self.num_of_particles)

        for i in range(len(self.particles)):

            if self.map_.is_collision([self.particles[i, 0], self.particles[i, 1]]) or self.map_.is_out_of_range([self.particles[i, 0], self.particles[i, 1]]):
                # replace odd particles with better ones
                self.particles[i] = self.particles[int(0.3 * len(self.particles))-1]
                continue

            if self.particles[i][2] > pi:
                self.particles[i][2] = -(2 * pi - self.particles[i][2])
            if self.particles[i][2] < -1 * pi:
                self.particles[i][2] = 2 * pi + self.particles[i][2]

    def generate_new_particle_coordinates(self):

        particle_x = self.map_x_range[0] + random() * (self.map_x_range[1] - self.map_x_range[0])
        particle_y = self.map_y_range[0] + random() * (self.map_y_range[1] - self.map_y_range[0])

        while self.map_.is_collision([particle_x, particle_y]) or self.map_.is_out_of_range([particle_x, particle_y]):
            particle_x = self.map_x_range[0] + random() * (self.map_x_range[1] - self.map_x_range[0])
            particle_y = self.map_y_range[0] + random() * (self.map_y_range[1] - self.map_y_range[0])

        return particle_x, particle_y

    def get_robot_readings(self):
        # update self.robot_readings with the sensor readings to use them while updating the weights
        self.robot_laser_val = laser_val if laser_val < 0.4 else 0.4

    def update_weights(self, R=0.0097):

        self.get_robot_readings()
        # print("robot laser val", self.robot_laser_val)
        for i in range(0, self.num_of_particles):

            # if self.particles[i, 3] == -20:
            #     continue

            particle_laser_val, particle_laser_intersect, particle_laser_range = self.map_.calc_particle_laser_val_and_intersect(
                x_p=self.particles[i][0], y_p=self.particles[i][1], theta_p=self.particles[i][2])

            self.particles[i, 3] = particle_laser_val

            # self.particles_laser_range[i] = particle_laser_range
            # self.particles_laser_intersect[i] = particle_laser_intersect

        self.weights.fill(1.)
        # the weight of each particle is proportional to the probability of the robot having the same sensor data as the particle
        self.weights *= scipy.stats.norm(self.particles[:, 3], 0.008).pdf(self.robot_laser_val)
        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= sum(self.weights)

        # print('weights:', self.weights)

    def systematic_resample(self):
        N = self.num_of_particles
        positions = (np.arange(N) + np.random.random()) / N
        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while (i < N and j < N):
            if (positions[i] < cumulative_sum[j]):
                indexes[i] = j
                i += 1
            else:
                j += 1

        return indexes

    def resample_from_index(self, indexes):
        num_of_random = int(0.08 * self.num_of_particles)
        self.particles[0:self.num_of_particles - num_of_random] = self.particles[
            indexes[0:self.num_of_particles - num_of_random]]
        self.weights[0:self.num_of_particles - num_of_random] = self.weights[
            indexes[0:self.num_of_particles - num_of_random]]

        self.particles[self.num_of_particles - num_of_random:self.num_of_particles, 0] = uniform(self.map_x_range[0],
                                                                                                 self.map_x_range[1],
                                                                                                 size=num_of_random)
        self.particles[self.num_of_particles - num_of_random:self.num_of_particles, 1] = uniform(self.map_y_range[0],
                                                                                                 self.map_y_range[1],
                                                                                                 size=num_of_random)
        self.particles[self.num_of_particles - num_of_random:self.num_of_particles, 2] = np.random.choice(
            [-pi / 2, pi / 2, pi, 0], size=num_of_random)

        for i in range(0, self.num_of_particles):
            particle_laser_val, particle_laser_intersect, particle_laser_range = self.map_.calc_particle_laser_val_and_intersect(
                x_p=self.particles[i][0], y_p=self.particles[i][1], theta_p=self.particles[i][2])

            self.particles[i][3] = particle_laser_val

    def particle_distance(self, particles):
        dist = np.zeros(len(particles))
        for i, particle in enumerate(particles):
            dist[i] = sum(np.sqrt((particle[0] - particles[:, 0]) ** 2 + (particle[1] - particles[:, 1]) ** 2))
        return particles[np.argmin(dist)]

    def new_resample(self):
        best_particles_idx = self.weights.argsort()[:int(0.4 * len(self.particles))]
        best_particle = self.particle_distance(self.particles[best_particles_idx])
        near_to_best_x = self.particles[np.where(np.abs(x - self.particles[:, 0]) <= 0.1)]
        near_to_best_y = near_to_best_x[np.where(np.abs(y - near_to_best_x[:, 1]) <= 0.1)]

        # calculate convergence percentage
        for i in range(0, 100):
            j = 100 - i
            if len(near_to_best_y) * 100 / self.num_of_particles >= j:
                self.percent_conv = j
                print('percent_conv:', j, '%')
                break

        print("best particle is at x = {}, y = {}".format(best_particle[0], best_particle[1]))

        random_size = len(self.particles) - len(best_particles_idx)
        random_valuable = int(0.8 * random_size)
        best_particles = self.particles[best_particles_idx]
        new_random_particles = np.empty((random_size - random_valuable, 4))
        new_random_valuable_particles = np.empty((random_valuable, 4))
        # kidnapping
        new_random_particles[:, 0] = uniform(self.map_x_range[0], self.map_x_range[1],
                                             size=random_size - random_valuable)
        new_random_particles[:, 1] = uniform(self.map_y_range[0], self.map_y_range[1],
                                             size=random_size - random_valuable)
        new_random_particles[:, 2] = np.random.choice([-pi / 2, pi / 2, pi, 0], size = random_size - random_valuable)
        # new_random_particles[:, 2] = np.random.choice([-1 * pi / 2 , 0, pi / 2, pi], size = random_size - random_valuable)
        # new_random_particles[:, 3] = self.get_particles_readings()

        indices_random_valuable = np.random.choice(best_particles.shape[0], size=(random_valuable))

        new_random_valuable_particles[:, 0] = uniform(-0.01, 0.01, size=random_valuable)
        new_random_valuable_particles[:, 1] = uniform(-0.01, 0.01, size=random_valuable)
        new_random_valuable_particles[:, 2] = uniform(-0.05, 0.05, size=random_valuable)
        # new_random_valuable_particles[:, 2] = np.random.choice([-pi / 2, pi / 2, pi, 0], size=random_valuable)

        new_random_valuable_particles += best_particles[indices_random_valuable]
        self.particles = np.concatenate([best_particles, new_random_valuable_particles, new_random_particles])

##################################################### Map Module #######################################################
class Map:

    def __init__(self, address):

        # Set address of map:
        self.map_address = address

        # Initialize a tree for map:
        self.tree = ET.parse(self.map_address)
        self.root = self.tree.getroot()

        # Initialize map parameters:
        self.rects_corners = []
        self.rects_centers = []
        self.rects_dimensions = []

        self.map_position = []

        self.all_edges = []
        self.rects_poly = []

        self.X_range = [0, 0]
        self.Y_range = [0, 0]

    def read_map(self):

        for links in self.root[0].iter('model'):
            try:
                for link in links:
                    tag = link.tag

                    if (tag == 'pose'):
                        pose = link.text.split(' ')
                        self.map_position = [float(pose[i]) for i in range(0, len(pose))]

            except Exception as error:
                pass

        for links in self.root[0].iter('model'):
            try:
                for link in links:
                    tag = link.tag

                    # Extract parameters of obstacles:
                    if (tag == 'link'):

                        # Initialize obstacle paramaters, assuming all obstacles are boxes:
                        rect_geometry = []
                        rect_pose = []

                        # Extract position of obstacle:
                        for _pose in link.iter('pose'):
                            pose = _pose.text.split(' ')
                            rect_pose = [float(pose[i]) + self.map_position[i] for i in range(0, len(pose))]
                            break  # The first pose found corresponds to collision position

                        # Extract dimensions of the edges of the obstacle:
                        for coll in link.iter('collision'):
                            geo = coll[3][0][0].text.split(' ')
                            rect_geometry = [float(geo[i]) for i in range(0, len(geo))]

                        # Calculate position of the corners of the obstacle based of pose and geometry data:
                        rect_corners = self.calc_rect_corners_from_edge_dimensions_and_rect_pose(rect_pose,
                                                                                                 rect_geometry)
                        # Calculate center of obstacle:
                        rect_center = [rect_pose[0], rect_pose[1]]
                        self.rects_dimensions.extend([[rect_geometry[0], rect_geometry[1]]])
                        # print(self.rects_dimensions)
                        # Add corners and center of current obstacle to that of other obstacles:
                        self.rects_corners.extend([rect_corners])
                        self.rects_centers.extend([rect_center])
            except Exception as error:
                # Link without 'collision' tag encountered -> geometry value left as None -> TypeError
                pass

        # Do other calculations:
        self.calc_all_edges_of_rects_using_corners()
        self.convert_corners_to_polygon()
        self.get_map_size()

    def is_out_of_range(self, point):

        point_x = point[0]
        point_y = point[1]

        if point_x < self.X_range[0] or point_x > self.X_range[1] or point_y < self.Y_range[0] or point_y > \
                self.Y_range[1]:
            return True
        else:
            return False

    def is_collision(self, point):

        p = Point(tuple(point))

        # Check if any of the polygons contain the point:
        collide = False
        for rect in self.rects_poly:
            if (rect.contains(p)):
                collide = True
                break

        return collide

    def calc_rect_corners_from_edge_dimensions_and_rect_pose(self, pose, dimensions):

        # Rectangle corners in clockwise direction (from top right):
        upper_right = [pose[0] + dimensions[0] * cos(pose[5]) / 2 - dimensions[1] * sin(pose[5]) / 2,
                       pose[1] + dimensions[0] * sin(pose[5]) / 2 + dimensions[1] * cos(pose[5]) / 2]
        lower_right = [pose[0] + dimensions[0] * cos(pose[5]) / 2 + dimensions[1] * sin(pose[5]) / 2,
                       pose[1] + dimensions[0] * sin(pose[5]) / 2 - dimensions[1] * cos(pose[5]) / 2]
        lower_left = [pose[0] - dimensions[0] * cos(pose[5]) / 2 + dimensions[1] * sin(pose[5]) / 2,
                      pose[1] - dimensions[0] * sin(pose[5]) / 2 - dimensions[1] * cos(pose[5]) / 2]
        upper_left = [pose[0] - dimensions[0] * cos(pose[5]) / 2 - dimensions[1] * sin(pose[5]) / 2,
                      pose[1] - dimensions[0] * sin(pose[5]) / 2 + dimensions[1] * cos(pose[5]) / 2]

        corners = [upper_right, lower_right, lower_left, upper_left]

        return corners

    def plot_map(self):

        ax = plt.axes()

        # Plot each rectangle and fill inside
        for rect_edges in self.all_edges:
            X = []
            Y = []
            for edge in rect_edges:
                x_ = [edge[0][0], edge[1][0]]
                y_ = [edge[0][1], edge[1][1]]

                X.extend(x_)
                Y.extend(y_)

            ax.fill(X, Y, c='gray', zorder=1)

    def convert_corners_to_polygon(self):

        self.rects_poly = []
        for rect in self.rects_corners:
            poly = Polygon([tuple(rect[0]), tuple(rect[1]), tuple(rect[2]), tuple(rect[3])])

            self.rects_poly.extend([poly])

    def calc_all_edges_of_rects_using_corners(self):

        self.all_edges = []
        for rect_corners in self.rects_corners:
            rect_edges = []
            rect_edges.extend([[rect_corners[1], rect_corners[0]]])
            rect_edges.extend([[rect_corners[0], rect_corners[3]]])
            rect_edges.extend([[rect_corners[3], rect_corners[2]]])
            rect_edges.extend([[rect_corners[2], rect_corners[1]]])

            self.all_edges.extend([rect_edges])

    # def get_map_size(self):

    #     x_range = []
    #     y_range = []
    #     for center in self.rects_centers:
    #         x_range.extend([center[0]])
    #         y_range.extend([center[1]])

    #     self.X_range = [min(x_range)+0.1, max(x_range)-0.1]
    #     self.Y_range = [min(y_range)+0.1, max(y_range)-0.1]

    def get_map_size(self):

        x_range = []
        y_range = []
        for center in self.rects_centers:
            x_range.extend([center[0]])
            y_range.extend([center[1]])

        x_min = min(x_range)
        x_min_index = x_range.index(x_min)

        y_min = min(y_range)
        y_min_index = y_range.index(y_min)

        x_max = max(x_range)
        x_max_index = x_range.index(x_max)

        y_max = max(y_range)
        y_max_index = y_range.index(y_max)

        self.X_range = [x_min + min(self.rects_dimensions[x_min_index]) / 2,
                        max(x_range) - min(self.rects_dimensions[x_max_index]) / 2]
        self.Y_range = [y_min + min(self.rects_dimensions[y_min_index]) / 2,
                        max(y_range) - min(self.rects_dimensions[y_max_index]) / 2]

        print('x_range:', self.X_range)
        print('y_range:', self.Y_range)

    def calc_particle_laser_val_and_intersect(self, x_p, y_p, theta_p):

        particle_laser_range = self.create_laser_range(x_=x_p, y_=y_p, theta_=theta_p)

        all_intersects = []
        particle_range_ = LineString([tuple(particle_laser_range[0]), tuple(particle_laser_range[1])])

        # since the laser range is up to a maximum of 40 cm, we draw a 40 cm line from the particle center in the direction of its heading
        for wall in self.all_edges:
            for line in wall:
                wall_ = LineString([tuple(line[0]), tuple(line[1])])
                intersect = wall_.intersection(particle_range_)
                if intersect:
                    all_intersects.append(intersect)

        min_dist, intersect_point = self.find_nearest_intersect(x_=x_p, y_=y_p, theta_=theta_p,
                                                                intersects=all_intersects)

        return min_dist, intersect_point, particle_laser_range

    def find_nearest_intersect(self, x_, y_, theta_, intersects):

        min_ = 0.4
        nearest = Point(x_ + 0.4 * cos(theta_), y_ + 0.4 * sin(theta_))
        for intersect in intersects:
            dist_ = sqrt(pow(intersect.x - x_, 2) + pow(intersect.y - y_, 2))
            if dist_ < min_:
                min_ = dist_
                nearest = intersect

        return min_, nearest

    def create_laser_range(self, x_, y_, theta_):

        # creating a line defining the range of the sensor reading for each particle
        starting_point = [x_, y_]
        ending_point = [x_ + 0.4 * cos(theta_), y_ + 0.4 * sin(theta_)]

        return [starting_point, ending_point]


######################################################### Read Map #################################################################
world_map = Map(address=map_address)

world_map.read_map()

world_map.calc_all_edges_of_rects_using_corners()


######################################################### Plot Current State #######################################################
def plot_current_state():
    plt.clf()

    world_map.plot_map()

    robot_arrow_length = 0.01
    # Calculate robot arrow values:
    dx = robot_arrow_length * cos(theta)
    dy = robot_arrow_length * sin(theta)

    plt.plot(pf.particles[:, 0], pf.particles[:, 1], 'o', c='black', markersize=0.7)
    plt.arrow(x - (dx / 2), y - (dy / 2), dx, dy, head_width=0.05, head_length=0.03, fc='r', ec='r', zorder=1)
    plt.draw()
    plt.pause(0.00000000000000000000000001)


######################################################### Update PF ################################################################
def update_particle_filter(d, dtheta_):
    pf.predict(d, dtheta_)
    pf.update_weights()
    # indexes = pf.systematic_resample()
    # pf.resample_from_index(indexes)
    pf.new_resample()


#################################### Handle the jump between positive and negative angles ##########################################
def handle_jump_between_positive_and_negative(init_angle, ref_angle, dir):
    new_ref_angle = ref_angle
    if (init_theta > 0 and ref_angle > 260 * pi / 180 and dir > 0):
        new_ref_angle = -(ref_angle - pi)
    elif (init_theta < 0 and ref_angle < -260 * pi / 180 and dir < 0):
        new_ref_angle = -(ref_angle + pi)

    return new_ref_angle


##################################################3# Handling small errors #########################################################
def handle_small_errors(init_angle, ref_angle, dir):
    new_ref_angle = ref_angle
    if (init_angle > 0 and ref_angle > pi and ref_angle < 2 * pi and ref_angle < 190 * pi / 180 and dir > 0):
        new_ref_angle = 2 * pi - ref_angle
    elif (init_angle < 0 and ref_angle < -pi and ref_angle > -190 * pi / 180 and dir < 0):
        new_ref_angle = -(2 * pi + ref_angle)

    return new_ref_angle


######################################################### Read Odometry ############################################################
x = 0.0
y = 0.0
theta = 0.0


def new_odometry(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


######################################################### Laser CallBack ###########################################################
laser_val = 100


def laser_callback(msg):
    global laser_val

    laser_val = msg.range
    global new_laser_data_flag,laser_data

    new_laser_data_flag = True
    laser_data = msg.range

######################################################### Init Rospy Node ##########################################################
rospy.init_node('main', anonymous=True)

######################################################### Init Publishers ##########################################################
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)

######################################################### Init Subscribers #########################################################
odometry_subscriber = rospy.Subscriber("/odom", Odometry, new_odometry)
sub = rospy.Subscriber('/vector/laser', Range, laser_callback)

######################################################### Other Initializations ####################################################
r = rospy.Rate(5)  # 5 Hz
vel_msg = Twist()

############ linear velocity ###########
linear_speed = 0.038
DISTANCE = 0.1
isForward = 1

current_distance = 0

linear_move_ongoing = True

############ Angular Velocity ###########
angular_speeds = np.array([15, -15, -15, -15])
angles = np.array([90, 0, -90, -180])
clockwise = 1

# Converting from angles to radians
angular_speeds = angular_speeds * 2 * pi / 360
relative_angles = angles * 2 * pi / 360

current_angular_move = 0
angular_move_ongoing = False

theta_segment = 45 * pi / 180

############# Initial Settings ###########
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

############# Main Loop ###################

tr_model_mean = 0.001376
tr_model_var = 0.00025222
tr_model = [tr_model_mean, tr_model_var]

rot_model_mean = 0.00298
rot_model_var = 0.07558
rot_model = [rot_model_mean, rot_model_var]

sn_model_mean = 0.05487407390773296
sn_model_var = 0.00006563749530838329
sn_model = [sn_model_mean, sn_model_var]

pf = ParticleFilter(map_=world_map, num_of_particles=900, translation_model=tr_model, rotation_model=rot_model,
                    sensor_model=sn_model)

x = 0.0
y = 0.0
theta = 0.0

last_x = 0
last_y = 0

get_last_loc_flag = True
random_move_state = True
yaw_setpoint = 0
r_setpoint = 0
r_max_p_term = 0.5

laser_data = 0
new_laser_data_flag = False

particle_number = 900

laser_range = 0.04

loop_iter = 0

is_halt = False
linear_movement_done = False

while not rospy.is_shutdown():
    plot_current_state()
    if random_move_state:

        #   choose a random move for vector
        #   which is acceptable for map
        yaw_setpoint = np.random.choice([-90, 90, 180, 0])

        r_setpoint = np.random.choice([0, 0.1, 0.15, 0.2, 0.25, 0.30, 0.35])

        # print('target = ', r_setpoint, yaw_setpoint)

        yaw_setpoint = yaw_setpoint * pi / 180.0
        def_rot = yaw_setpoint - theta

        random_move_state = False
        loop_iter = 0

        robot_movment_complite_flag = False

        linear_movement_done = False

    else:

        if not robot_movment_complite_flag:
            #   calculte the yaw error
            yaw_error = yaw_setpoint - theta
            vel_msg.angular.z = yaw_error * 0.8

            if yaw_error < 0.01 and yaw_error > -0.01:

                #   stop the rotation
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                time.sleep(0.1)

                #   just do it once
                loop_iter += 1
                if loop_iter == 1:
                    # wait for new laser data
                    new_laser_data_flag = False
                    while not new_laser_data_flag: pass
                    last_laser_data = laser_data

                if last_laser_data > r_setpoint + 0.03:
                    # save the first place location
                    if get_last_loc_flag:
                        last_x = x
                        last_y = y
                        get_last_loc_flag = False
                    # calculate the transition error
                    r_error = r_setpoint - sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
                    r_p_term = r_error * 1
                    # limit the transition speed
                    if r_p_term > r_max_p_term:
                        r_p_term = r_max_p_term
                    # move linear
                    vel_msg.linear.x = r_p_term * 1
                    if r_error < 0.01 and r_error > -0.01:
                        vel_msg.linear.x = 0
                        velocity_publisher.publish(vel_msg)
                        get_last_loc_flag = True
                        robot_movment_complite_flag = True

                    linear_movement_done = True

                else:
                    robot_movment_complite_flag = True

            velocity_publisher.publish(vel_msg)

        # ------------------------------------------------------------
        # ++++++++++++++++++++ predict (move) particles ++++++++++++++
        #   if the robot in gazebo stoped then do rest of things
        else:
            if not is_halt:
                # print('move particle')
                e_rot_in_rot = np.random.normal(def_rot, 0.0005, particle_number)
                total_transition = 0
                plot_current_state()

                if linear_movement_done:
                    d = r_setpoint
                else:
                    d = 0
                pf.predict(d, def_rot, yaw_setpoint)
                pf.update_weights()
                pf.new_resample()

                random_move_state = True