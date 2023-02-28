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

"""
1. import map similar to those used by ROS (stored in a common image format)
You can build a map yourself (which is harder), if you like, or use the
one that is loaded into the simulator (which is easier).

2. Implement MCL with a fixed number of particles,
	and a way of visualizing these particles.

3. You should implement the ability to return the most likely pose at any given time.
There are several ways to do this, but returning the current most probable pose is simplest.
"""

class MarkovLoc:
	def __init__(self, map_path):
		# initialize node
		rospy.init_node('avoid_wall', anonymous=True)

		assert Path(map_path).is_file()
		self.map_img = np.asarray(PImage.open(map_path))

		self.thetas = 360

		# init belief
		self.belief = None
		self.init_belief()

		# subscribe to lidar
		sub = rospy.Subscriber('base_scan', LaserScan, self.lidar_callback)
		self.pub_map = rospy.Publisher('belief_map', Image, queue_size=10)
		self.pub_loc = rospy.Publisher('loc', PoseWithCovariance, queue_size=10)

		self.lidar_angles = []
		self.lidar_ranges = []

		self.bridge = CvBridge()

	def init_belief(self):
		assert len(self.map_img.shape) == 2
		width, height, thetas = *self.map_img.shape, 360
		self.belief = np.ones((width, height, thetas))
		self.belief *= np.stack([self.map_img]*thetas, axis=2)
		self.belief *= (1/np.count_nonzero(self.map_img))

	def init_lidar_angles(self, msg):
		min, max = int(degrees(msg.angle_min)), int(degrees(msg.angle_max))
		self.lidar_angles = list(range(min, max))
		assert len(self.lidar_angles) == len(msg.ranges)

	def lidar_callback(self, msg):
		if len(self.lidar_angles) == 0:
			self.init_lidar_angles(msg)
		self.msg_lidar = msg
		
	def odom_callback(self, msg):
		self.msg_odom = msg

	def act(self):
		"""
		p(loc_t | odom) = integral( p(loc_t | l'_t-1, o_t)p(l'_t-1)) dl'_t-1

		...the total probabliity for a specific location l is built up from the individual
		contributions from every location l' in the former belief state given the odom
		"""
		#	map from a belief state and action to a new predicted belief state
		#	sum over all possible ways in which the robot may have reached state
		for x in range(self.belief.shape[0]):
			for y in range(self.belief.shape[1]):
				for theta in range(self.thetas):
					# TDOO: do prediction step
					continue

	def see(self):
		"""
		p(loc | sense) = p(sense | loc) p(l)
		"""
		for x in range(self.belief.shape[0]):
			for y in range(self.belief.shape[1]):
				for theta in range(self.thetas):
					p_sense_loc = 1 # ??
					self.belief[x, y, theta] *= p_sense_loc
	
	def pub_max_loc(self):
		_flat_idx = np.argmax(self.belief)
		self.curr_max_idx = np.unravel_index(_flat_idx, self.belief.shape)
		max_prob_loc = PoseWithCovariance()
		# max_prob_loc.covariance = [self.belief[self.curr_max_idx]]
		max_prob_loc.pose.position.x = self.curr_max_idx[0]
		max_prob_loc.pose.position.y = self.curr_max_idx[1]
		max_prob_loc.pose.orientation.z = radians(self.curr_max_idx[2])
		self.pub_loc.publish(max_prob_loc)

	def pub_map_img(self):
		cv2_img = self.belief[:, :, self.curr_max_idx[2]]
		cv2_img[self.curr_max_idx[0], self.curr_max_idx[1]] = 1 
		cv2_img = (cv2_img*255).astype('uint8')
		ros_img = self.bridge.cv2_to_imgmsg(cv2_img)
		self.pub_map.publish(ros_img)

	def run(self):
		while not rospy.is_shutdown():
			self.see()
			self.act()
			self.pub_max_loc()
			self.pub_map_img()

if __name__ == '__main__':
	# get private parameters set in launch file
	# map_path = rospy.get_param('~map')
	map_path = '/home/nathan/catkin_ws/src/rob551/stage_osu/config/simple_rooms.png'
	ml = MarkovLoc(map_path)
	ml.run()