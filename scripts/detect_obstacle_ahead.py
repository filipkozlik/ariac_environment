#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobotControl(object):

	def __init__(self, distance, sub_topic, pub_topic, angle_scan, vel_topic):
		print('INIT')
		self.distance = distance
		self.angle_scan = angle_scan
		self.ranges_size = None

		self.front_scan_min = None
		self.back_scan_min = None
		self.front_scan_max = None
		self.back_scan_max = None

		self.forwards = None

		rospy.init_node('control_robot_node', anonymous=True)
		rospy.Subscriber(sub_topic, LaserScan, self.check_obstacle)
		rospy.Subscriber(vel_topic, Twist, self.check_direction)
		self.pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)

		rospy.spin()

	def check_direction(self, twist):
		if twist.linear.x > 0:
			self.forwards = True
		elif twist.linear.x < 0:
			self.forwards = False
		else:
			self.forwards = None

	def check_obstacle(self, scan_data):
		if self.front_scan_min == None:
			angle_range = int(self.angle_scan/(2*np.rad2deg(scan_data.angle_increment)))
			self.ranges_size = len(scan_data.ranges)
			front_scan = -scan_data.angle_min*1.0/(scan_data.angle_max - scan_data.angle_min) * self.ranges_size
			self.front_scan_min = int(front_scan) - angle_range
			self.front_scan_max = int(front_scan) + angle_range
			self.back_scan_min = self.ranges_size - angle_range
			self.back_scan_max = 0 + angle_range

		# check if robot is moving forwards or backwards
		if self.forwards == True:
			print('forwards')
			for scan in range(self.front_scan_min, self.front_scan_max + 1):
				if scan_data.ranges[scan] < self.distance:
					print('Obstacle in ' + str(self.distance) + ' meters')
					break
				elif scan_data.ranges[scan] < 2*self.distance:
					print('Obstacle in ' + str(2*self.distance) + ' meters')
					break
				elif scan_data.ranges[scan] < 4*self.distance:
					print('Obstacle in ' + str(4*self.distance) + ' meters')
					break
		elif self.forwards == False:
			print('backwards')
		else:
			pass

if __name__ == '__main__':
    try:
    	vel_topic = sys.argv[1]
        distance = float(sys.argv[2])
        sub_topic = sys.argv[3]
        pub_topic = sys.argv[4]
        angle_scan = float(sys.argv[5]) # in degrees
        control = RobotControl(distance, sub_topic, pub_topic, angle_scan, vel_topic)
    except rospy.ROSInterruptException:
        pass