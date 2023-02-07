#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import tanh


def callback(data):

	dist = data.ranges[0]

	vel = (tanh(dist-0.3)*0.5)
	vel -= vel%0.01

	t = Twist()
	t.linear.x = max(0.0, vel)
	t.linear.y = 0.0
	t.linear.z = 0.0
	t.angular.x = 0.0
	t.angular.y = 0.0
	t.angular.z = 0.0
	publisher.publish(t)

	rospy.loginfo(f'dist = {dist}')


if __name__ == '__main__':

	rospy.init_node('avoid_wall', anonymous=True)

	# setup publisher to publish velocity commands
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# subscribe to lidar
	rospy.Subscriber('scan', LaserScan, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
