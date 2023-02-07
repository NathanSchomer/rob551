#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import tanh


def callback(data):

	mid_idx = len(data.ranges)//2
	dist = data.ranges[mid_idx]

	t = Twist()
	t.linear.x = tanh(dist-0.4)
	t.linear.y = 0.0
	t.linear.z = 0.0
	t.angular.x = 0.0
	t.angular.y = 0.0
	t.angular.z = 0.0

	# Send the command to the robot.
	publisher.publish(t)
	rospy.loginfo(f'min range = {min(data.ranges)}')


if __name__ == '__main__':

	rospy.init_node('avoid_wall', anonymous=True)

	# setup publisher to publish velocity commands
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# subscribe to lidar
	rospy.Subscriber('base_scan', LaserScan, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
