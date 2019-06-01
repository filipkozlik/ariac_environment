#!/usr/bin/env python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobotControl(object):

	def __init__(self, distance, sub_topic, angle_scan):
		self.distance = distance
		self.angle_scan = angle_scan
		self.ranges_size = None

		self.front_scan_min = None
		self.back_scan_min = None
		self.front_scan_max = None
		self.back_scan_max = None

		rospy.init_node('control_robot_node', anonymous=True)
		rospy.Subscriber(sub_topic, LaserScan, self.check_obstacle)
#		self.pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)

		rospy.spin()

	def check_obstacle(self, scan_data):
		if self.front_scan_min == None:
			angle_range = int(self.angle_scan/(2*np.rad2deg(scan_data.angle_increment)))
			self.ranges_size = len(scan_data.ranges)
			front_scan = -scan_data.angle_min*1.0/(scan_data.angle_max - scan_data.angle_min) * self.ranges_size
			self.front_scan_min = int(front_scan) - angle_range
			self.front_scan_max = int(front_scan) + angle_range
			self.back_scan_min = self.ranges_size - angle_range
			self.back_scan_max = 0 + angle_range


		obstacles_found = []
		obstacle_in_progress = False

		for scan in range(self.front_scan_min, self.front_scan_max):
			if 0.95 < (scan_data.ranges[scan + 1]/scan_data.ranges[scan]) < 1.05:
				if obstacle_in_progress == False:
					obstacle_in_progress = True
					obstacles_found.append([scan, scan+1])
			else:
				if obstacle_in_progress == True:
					obstacle_in_progress = False
					obstacles_found[-1][1] = scan

		print obstacles_found

		obstacles_found_lines = []

		for ob in obstacles_found:
			dist = (scan_data.ranges[ob[0]], scan_data.ranges[ob[1]])
			ang = (ob[0]*scan_data.angle_increment - np.pi, ob[1]*scan_data.angle_increment - np.pi)
			points = []
			points.append(((dist[0]*np.cos(ang[0]) + 9.73, dist[0]*np.sin(ang[0]) + 9.58), (dist[1]*np.cos(ang[1]) + 9.73, dist[1]*np.sin(ang[1]) + 9.58)))
			obstacles_found_lines.append(points)

		print obstacles_found_lines



if __name__ == '__main__':
    try:
        distance = float(sys.argv[1])
        sub_topic = sys.argv[2]
        angle_scan = float(sys.argv[3]) # in degrees
        control = RobotControl(distance, sub_topic, angle_scan)
    except rospy.ROSInterruptException:
        pass