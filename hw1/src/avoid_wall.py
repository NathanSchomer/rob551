#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import tanh


def callback(data):
	"""
	Callback for everytime new lidar data is received
	"""
	dist = data.ranges[ranges_idx]			# get lidar reading from directly in from of robot

	vel = (tanh(dist-stop_dist)*max_vel)	# use tanh to slow down nicely
	vel -= vel%0.01							# round to nearest hundreth

	t = Twist()								# publish new velocity
	t.linear.x = max(0.0, vel)
	publisher.publish(t)


if __name__ == '__main__':

	rospy.init_node('avoid_wall', anonymous=True)

	stop_dist = rospy.get_param('~stop_dist')
	max_vel = rospy.get_param('~max_vel')
	ranges_idx = rospy.get_param('~ranges_idx')
	lidar_topic = rospy.get_param('~lidar_topic')
	vel_topic = rospy.get_param('~vel_topic')

	# setup publisher to publish velocity commands
	publisher = rospy.Publisher(vel_topic, Twist, queue_size=10)

	# subscribe to lidar
	rospy.Subscriber(lidar_topic, LaserScan, callback)
 
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()