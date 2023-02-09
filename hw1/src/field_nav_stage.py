#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, tanh, sqrt, acos, degrees
from geometry_msgs.msg import Twist

def calc_field(data, fov=0.5*pi, zero_dist=4):
    new_ranges = []
    angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
    for theta, dist in zip(angles, data.ranges):
        if abs(theta)%2*pi <= fov/2:
            new_ranges.append((theta, dist-zero_dist))
    return new_ranges

def calc_move(data, new_ranges):
    xs = []
    ys = []
    for theta, dist in new_ranges:
        xs.append(dist * cos(theta))
        ys.append(dist * sin(theta))
    x, y = np.mean(xs), np.mean(ys)
    mag = sqrt(x**2 + y**2)
    theta = acos(x/mag)
    return (theta, mag)

def listener(data):

    new_ranges = calc_field(data)
    theta, mag = calc_move(data, new_ranges)

    deg_theta = degrees(theta)

    t = Twist()
    t.linear.x = mag
    t.linear.y = 0
    t.linear.z = 0.0
    t.angular.x = 0.0
    t.angular.y = 0.0
    t.angular.z = theta*0.25
    rospy.loginfo(f'Published theta = {t.angular.z}')

    cmd_vel.publish(t)

if __name__ == '__main__':
    rospy.init_node('field_nav_stage')
    rospy.Subscriber('/base_scan', LaserScan, listener)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()