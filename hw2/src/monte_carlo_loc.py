#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import tanh
import numpy as np
from PIL import Image
from pathlib import Path

"""
1. import map similar to those used by ROS (stored in a common image format)
You can build a map yourself (which is harder), if you like, or use the
one that is loaded into the simulator (which is easier).

2. Implement MCL with a fixed number of particles,
	and a way of visualizing these particles.

3. You should implement the ability to return the most likely pose at any given time.
There are several ways to do this, but returning the current most probable pose is simplest.
"""


def callback(data):
	"""
	Callback for everytime new lidar data is received
	"""
	return


if __name__ == '__main__':

	# initialize node
	rospy.init_node('avoid_wall', anonymous=True)

	# get private parameters set in launch file
	map_path = rospy.get_param('~map')
	# lidar_topic = rospy.get_param('~lidar_topic')

	assert Path(map_path).is_file()
	map_img = np.asarray(Image.open(map_path))

	rospy.logdebug(f'Loaded map image of size {map_img.shape}')

	# setup publisher to publish velocity commands
	# publisher = rospy.Publisher(vel_topic, Twist, queue_size=10)

	# subscribe to lidar
	# rospy.Subscriber(lidar_topic, LaserScan, callback)
 
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()