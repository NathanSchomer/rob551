#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, tanh, sqrt, acos, degrees, isinf
from geometry_msgs.msg import Twist

ignore_dist = 0.75
dist_exp = 2
forward_vec = 0.75
max_ang_rate = 2
vel_scale = 0.75
ang_scale = 0.25
fov = pi/2
lidar_topic = 'base_scan'

def calc_obs_components(data, fov=pi/2, max_dist=1):
    new_ranges = []
    angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
    for theta, dist in zip(angles, data.ranges):
        if isinf(dist) or dist > max_dist:
            continue
        theta *= -1
        new_ranges.append((theta, dist))
    return new_ranges

def calc_obs_vector(data, new_ranges):
    xs, ys = [], []
    for theta, dist in new_ranges:
        if dist == 0:
            dist += 0.0000001
        dist = 1/(dist**dist_exp)
        xs.append(dist * cos(theta))
        ys.append(dist * sin(theta))
    x, y = np.sum(xs), np.sum(ys)
    return x, y

def listener(data):

    new_ranges = calc_obs_components(data, fov=fov,
                                     max_dist=ignore_dist)
    x, y = calc_obs_vector(data, new_ranges)

    x += forward_vec
    mag = sqrt(x**2 + y**2)
    theta = acos(x/mag)
    if y < 0:
        theta *= -1

    if abs(theta) > max_ang_rate:
        theta = (theta/abs(theta))*max_ang_rate

    t = Twist()
    t.linear.x = tanh(mag)*vel_scale
    t.angular.z = theta*ang_scale
    rospy.loginfo(f'Published theta = {t.angular.z}')

    cmd_vel.publish(t)

if __name__ == '__main__':

    rospy.init_node('field_nav_stage')
    rospy.Subscriber(lidar_topic, LaserScan, listener)

    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()