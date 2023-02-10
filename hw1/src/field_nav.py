#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, tanh, sqrt, acos, degrees, isinf
from geometry_msgs.msg import Twist

def calc_obs_components(data, fov=pi/2, max_dist=1):
    new_ranges = []
    angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
    for theta, dist in zip(angles, data.ranges):
        if (isinf(dist) or dist > max_dist) or\
            (invert_theta and not (pi-fov/2) < theta < (pi+fov/2)) or\
            (not invert_theta and (fov/2) < theta < (2*pi-fov/2)):
            continue
        theta *= -1
        new_ranges.append((theta, dist))
    return new_ranges

def calc_obs_vector(data, new_ranges):
    xs, ys = [], []
    for theta, dist in new_ranges:
        dist = max(abs(dist-0.1), 0.0000001)
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
    theta = acos(x/mag) * ang_scale
    theta = min(theta, max_ang_rate)

    if invert_theta and y > 0:
        theta *= -1
    elif not invert_theta and y < 0:
        theta *= -1

    t = Twist()
    dist = ()
    mid_idx = len(data.ranges)//2
    front_dist = np.mean(data.ranges[mid_idx-3:mid_idx+3])
    vel = (tanh(front_dist-0.3)*0.5)
    vel -= vel%0.01
    t.linear.x = forward_vec
    t.angular.z = theta
    rospy.loginfo(f'Published theta = {t.angular.z}')

    cmd_vel.publish(t)

if __name__ == '__main__':

    rospy.init_node('field_nav_stage', anonymous=True)

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

# #!/usr/bin/env python3
# import numpy as np
# import rospy
# from sensor_msgs.msg import LaserScan
# from math import sin, cos, pi, tanh, sqrt, acos, degrees, isinf
# from geometry_msgs.msg import Twist

# from time import time

# def calc_obs_components(data, max_dist=1):
#     new_ranges = []
#     angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
#     for theta, dist in zip(angles, data.ranges):
#         if isinf(dist) or dist > 0.5:
#             continue
#         theta = (theta + pi) % 2*pi # flip the angle to get the opposing vector
#         new_ranges.append((theta, dist))
#     return new_ranges

# def calc_obs_vector(data, new_ranges):
#     xs = []
#     ys = []
#     for theta, dist in new_ranges:
#         xs.append((1-tanh(dist/4)) * cos(theta))
#         ys.append((1-tanh(dist/4)) * sin(theta))
#     x, y = np.sum(xs), np.sum(ys)
#     return x, y

# def listener(data):

#     new_ranges = calc_obs_components(data)
#     x, y = calc_obs_vector(data, new_ranges)

#     x += 1
#     mag = sqrt(x**2 + y**2)
#     theta = acos(x/mag)

#     if y < 0:
#         theta *= -1

#     t = Twist()
#     t.linear.x = tanh(mag)*0.4
#     t.angular.z = tanh(theta)*.8
#     rospy.loginfo(f'Published theta = {t.angular.z}')

#     cmd_vel.publish(t)

# if __name__ == '__main__':

#     rospy.init_node('field_nav')
#     rospy.Subscriber('/scan', LaserScan, listener)

#     cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     rospy.spin()