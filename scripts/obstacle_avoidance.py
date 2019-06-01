#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion as efq
import matplotlib.pyplot as plt


class ObstacleAvoidance(object):

	def __init__(self):
		sub_topic = '/pioneer/front_laser'
		pub_topic = '/pioneer/path'
		self.aaa = False
		self.robot_radius = 0.4

		rospy.init_node('obstacle_avoidance', anonymous=True)
		rospy.Subscriber(sub_topic, LaserScan, self.avoidance_algorithm)
		self.pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)

		rospy.spin()

	def avoidance_algorithm(self, data):
		# check if obstacle in path
		obstacle_in_path, radius, distance = self.is_obstacle_in_path()
		if obstacle_in_path:
			is_safe = self.safety_check()
			if is_safe:
				avoidance_path = self.get_avoid_path(radius, distance)
				realworld_path = self.convert_path_to_world(avoidance_path)
				self.plot_avoidance_path(realworld_path)

	# check if obstacle is in the way
	def is_obstacle_in_path(self):
		obstacle_radius = 0.3
		obstacle_distance = 1
		return True, obstacle_radius, obstacle_distance

	# check if is safe to avoid obstacle
	def safety_check(self):
		return True

	# make avoiding path
	def get_avoid_path(self, r, d):
		path_parts = []
		y_dist = r + self.robot_radius
		x_dist = d - self.robot_radius
		straight_part_length = 2*self.robot_radius + r
		path_parts.append(self.curve_path_part(x_dist, y_dist, False, 0))
		path_parts.append(self.straight_path_part(straight_part_length, y_dist, x_dist))
		path_parts.append(self.curve_path_part(x_dist, y_dist, True, straight_part_length + x_dist))
		return path_parts

	def straight_path_part(self, x_max, y_max, displacement):
		x = np.linspace(displacement, displacement+x_max, 100*x_max)
		y = [y_max]*int(100*x_max)
		return [x, y]
		
	def curve_path_part(self, x_max, y_max, reverse, displacement):
		x = [c*x_max for c in [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]]
		x = [c+displacement for c in x]
		if reverse:
			x = list(reversed(x))
		y = [c*y_max for c in [0, 0.01, 0.06, 0.16, 0.31, 0.5, 0.69, 0.84, 0.94, 0.99, 1]]
		p = np.polyfit(x, y, 5)
		xp = np.linspace(displacement, displacement + x_max, 100*x_max)
		yp = np.polyval(p, xp)
		return [xp, yp]

	def convert_path_to_world(self, path):
		try:
			model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			coords = model('robot_description', '')
			position = (coords.pose.position.x, coords.pose.position.y)

			orientation = coords.pose.orientation
			quaternions = [orientation.x, orientation.y, orientation.z, orientation.w]
			euler = efq(quaternions)

			path = [
				list(path[0][0])+list(path[1][0][1:])+list(path[2][0][1:]), 
				list(path[0][1])+list(path[1][1][1:])+list(path[2][1][1:])
			]

			target_frame = '/pioneer/map'
			current_frame = '/pioneer/base_link'

			tf_buff = tf2_ros.Buffer()
			tf_listener = tf2_ros.TransformListener(tf_buff)
			tf = tf_buff.lookup_transform(target_frame, current_frame, rospy.Time(0))
			realworld_path = tf2_geometry_msgs.do_transform_pose(path, tf)

			print realworld_path

		except rospy.ServiceException as exc:
			rospy.loginfo('Couldn\'t get model state: {}'.format(exc))

		return path

	def plot_avoidance_path(self, p):
		if self.aaa == False:
			plt.plot(p[0], p[1])
			plt.show()
			self.aaa = True

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass