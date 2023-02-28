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

		self.bridge = CvBridge()

		# init particles
		self.num_particles = 10
		self.state_dim = 3
		# [prob, x, y, theta]_0 ... []_M
		self.particles = np.ones([self.state_dim+1, self.num_particles])/self.num_particles

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

	def motion_update(self):
		return

	def sensor_update(self):
		return
	
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

	def redistribute_particles(self):
		for m, particle in enumerate(self.particles):
			# draw x_t^[m] from X_t with prob w_t^[m]
			# X_t += x_t^[m]
			continue
		return

	def run(self):
		"""
		main loop
		"""
		while not rospy.is_shutdown():
			for m in self.particles:
				self.motion_update()
				self.sensor_update()
			self.redistribute_particles()
			# self.pub_max_loc()
			# self.pub_map_img()

if __name__ == '__main__':
	# get private parameters set in launch file
	# map_path = rospy.get_param('~map')
	map_path = '/home/nathan/catkin_ws/src/rob551/stage_osu/config/simple_rooms.png'
	ml = MarkovLoc(map_path)
	ml.run()