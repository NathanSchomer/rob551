#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, tanh, sqrt, acos, degrees, isinf
from geometry_msgs.msg import Twist

def calc_obs_components(data, fov=pi/2, max_dist=1):
    """
    Filter lidar data to desired FOV and max_dist
        and convert measurements to polar coordinates
    """
    new_ranges = []

    # calculate angles for each lidar data point
    angles = np.linspace(data.angle_min,
                         data.angle_max, 
                         len(data.ranges))

    for theta, dist in zip(angles, data.ranges):
        # filter lidar data with FOV and max_dist
        if (isinf(dist) or dist > max_dist) or\
            (invert_theta and not (pi-fov/2) < theta < (pi+fov/2)) or\
            (not invert_theta and (fov/2) < theta < (2*pi-fov/2)):
            continue
        # invert angle
        theta *= -1
        new_ranges.append((theta, dist))
    return new_ranges

def calc_obs_vector(data, new_ranges):
    """
    Use filtered lidar data to calculate obstacle vector
    """
    xs, ys = [], []
    for theta, dist in new_ranges:
        dist = max(abs(dist-0.1), 0.0000001)    # dist can never be 0 b/c of next line...
        dist = 1/(dist**dist_exp)
        xs.append(dist * cos(theta))
        ys.append(dist * sin(theta))
    x, y = np.sum(xs), np.sum(ys)
    return x, y

def listener(data):

    new_ranges = calc_obs_components(data, fov=fov,
                                     max_dist=ignore_dist)
    x, y = calc_obs_vector(data, new_ranges)

    x += forward_vec                    # add forward vector to obstacle vector
    mag = sqrt(x**2 + y**2)             # calculate magnitude of the vector
    theta = acos(x/mag) * ang_scale     # calculate angle of the vector
    theta = min(theta, max_ang_rate)

    # Physical Turtlebot needs theta inverted
    #   (I mounted my lidar backwards)
    if invert_theta and y > 0:
        theta *= -1
    elif not invert_theta and y < 0:
        theta *= -1

    # publish velocity and angle
    t = Twist()
    dist = ()
    t.linear.x = forward_vec
    t.angular.z = theta
    cmd_vel.publish(t)

if __name__ == '__main__':

    rospy.init_node('field_nav_stage', anonymous=True)

    # get private ros params set in launch file
    ignore_dist = rospy.get_param('~ignore_dist')
    dist_exp = rospy.get_param('~dist_exp')
    forward_vec = rospy.get_param('~forward_vec')
    max_ang_rate = rospy.get_param('~max_ang_rate')
    vel_scale = rospy.get_param('~vel_scale')
    ang_scale = rospy.get_param('~ang_scale')
    fov = rospy.get_param('~fov')
    lidar_topic = rospy.get_param('~lidar_topic')
    zero_dist = rospy.get_param('~zero_dist')
    invert_theta = rospy.get_param('~invert_theta', default=False)

    rospy.Subscriber(lidar_topic, LaserScan, listener)

    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()
