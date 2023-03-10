#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from math import tanh
import numpy as np
from PIL import Image as PImage
from pathlib import Path
from math import radians, degrees
from cv_bridge import CvBridge, CvBridgeError
import cv2
from math import cos, sin, inf
from tf.transformations import euler_from_quaternion


class HIMM:
    def __init__(self, pose_topic, lidar_topic, map_dim, pose_offset, scale_factor, max_range):

        # load map
        self.map_dims = (map_dim, map_dim)

        # initialize map to all zeros
        self.map = np.zeros(self.map_dims, dtype=np.uint8)

        # subscribe to lidar and pose data
        self.pose = None
        self.sub_pose = rospy.Subscriber(pose_topic, Odometry, self.pose_callback) # TODO: this type might be wrong
        self.sub_lidar = rospy.Subscriber(lidar_topic, LaserScan, self.lidar_callback)
        self.pub_map = rospy.Publisher('map', Image, queue_size=10)

        self.INC_VAL, self.DEC_VAL = 3, -1
        self.CELL_MIN, self.CELL_MAX = 0, 15
        self.POSE_OFFSET = pose_offset
        self.SCALE_FACTOR = scale_factor
        self.MAX_RANGE = max_range

        self.thetas = []

        self.bridge = CvBridge()

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        self.pose = (round((x+self.POSE_OFFSET)*self.SCALE_FACTOR),
                     round((y+self.POSE_OFFSET)*self.SCALE_FACTOR),
                     yaw) 

    def mod_map(self, idxs: set, val: int):
        for idx in idxs:
            new_val = int(self.map[idx]) + val
            if new_val > self.CELL_MAX:
                new_val = self.CELL_MAX
            if new_val < self.CELL_MIN:
                new_val = self.CELL_MIN
            self.map[idx] = new_val

    def init_thetas(self, data):
        self.thetas = np.linspace(data.angle_min,
                                  data.angle_max, 
                                  len(data.ranges))

    def inc_map(self, idxs: set):
        self.mod_map(idxs, self.INC_VAL)

    def dec_map(self, idxs: set):
        self.mod_map(idxs, self.DEC_VAL)

    def calc_endpoint(self, lidar_theta, range, pose):
        theta = lidar_theta + pose[2]
        x = pose[0] + int(cos(theta)*range)
        y = pose[1] + int(sin(theta)*range)
        return (x, y)

    def calc_empty_cells(self, pose, endpoint):
        # use https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm to calc empty cells
        idxs = set()
        x0, y0 = pose[0:2]
        x1, y1 = endpoint

        if x1 < x0:
            x0, x1 = x1, x0
            y0, y1 = y1, y0

        dx, dy = x1-x0, y1-y0
        D = 2*dy - dx
        y = y0

        for x in range(x0, x1):
            if x == x0 or x == x1:
                continue
            idxs.add((x, y))
            if D > 0:
                y += 1
                D -= 2*dx
            D += 2*dy
        return idxs

    def publish_map(self):
        img = self.map.astype(np.uint8)
        img[img == 0] = 255
        img[img == 15] = 0
        img = np.stack([img]*3, axis=-1)
        idxs = np.transpose(np.where(np.any(0 < img, axis=2) & 
                                     np.any(img < 255, axis=2)))
        for idx in idxs:
            img[idx[0], idx[1], 1:2] = 0
            img[idx[0], idx[1], 0] = 255

        # TODO: flip map upside down... and maybe sideways?

        # cv2.imwrite('/media/psf/Home/Desktop/img.png', img)
        ros_img = self.bridge.cv2_to_imgmsg(img)
        self.pub_map.publish(ros_img)

    def lidar_callback(self, lidar_data):
        if len(self.thetas) == 0:
            self.init_thetas(lidar_data)
        pose = self.pose
        inc_idxs, dec_idxs = set(), set()
        for theta, curr_range in zip(self.thetas, lidar_data.ranges):
            curr_range *= self.SCALE_FACTOR
            if curr_range == inf:
                continue
            endpoint = self.calc_endpoint(theta, curr_range, pose)
            if curr_range < self.MAX_RANGE * self.SCALE_FACTOR:
                inc_idxs.add(endpoint)
            dec_idxs.union(self.calc_empty_cells(pose, endpoint))
        self.inc_map(inc_idxs)
        self.dec_map(dec_idxs)
        self.publish_map()


if __name__ == '__main__':

    rospy.init_node('himm', anonymous=True)

    pose_topic = rospy.get_param('~pose_topic', default='/base_pose_ground_truth')
    lidar_topic = rospy.get_param('~lidar_topic', default='/base_scan')
    map_dim = rospy.get_param('~map_dim', default=1000)
    pose_offset = rospy.get_param('~pose_offset', default=8)
    scale_factor = rospy.get_param('~scale_factor', default=10)
    max_range = rospy.get_param('~max_range', default=np.inf)

    himm = HIMM(pose_topic, lidar_topic, map_dim, pose_offset, scale_factor, max_range)
    rospy.spin()
