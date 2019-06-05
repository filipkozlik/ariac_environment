#!/usr/bin/env python

import tf as tfi
import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import GetModelState
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion as efq
import matplotlib.pyplot as plt


class ObstacleAvoidance(object):

	def __init__(self):

		sub_topic = '/pioneer/front_laser'
		pub_topic = '/pioneer/path'
		self.aaa = False
		self.robot_radius = 0.4

		while True:
			global_path = self.get_path()
			if not global_path == None:
				break

		rospy.init_node('obstacle_avoidance', anonymous=True)
		rospy.Subscriber(sub_topic, LaserScan, self.avoidance_algorithm)
		self.pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)

		rospy.spin()

	def get_path(self):
		path = None
		try:
			path = rospy.ServiceProxy('retrieve_path', GetModelState)
			print path.call()

		except rospy.ServiceException as exc:
			rospy.loginfo('Couldn\'t get path: {}'.format(exc))

		return path

	def avoidance_algorithm(self, data):
		# check if obstacle in path
		obstacle_in_path, radius, distance = self.is_obstacle_in_path()
		if obstacle_in_path:
			is_safe = self.safety_check()
			if is_safe:
				avoidance_path = self.get_avoid_path_points(radius, distance)
				realworld_path, pos = self.convert_path_points_to_world(avoidance_path)
				# self.plot_avoidance_path(realworld_path)

	# check if obstacle is in the way
	def is_obstacle_in_path(self):
		obstacle_radius = 0.3
		obstacle_distance = 1
		return True, obstacle_radius, obstacle_distance

	# check if is safe to avoid obstacle
	def safety_check(self):
		return True

	# make avoiding path points
	def get_avoid_path_points(self, r, d):
		path_points = []
		y_dist = 2*r + self.robot_radius
		x_dist = d - self.robot_radius
		straight_part_length = 2*self.robot_radius + 2*r

		path_points.append([x_dist, 0])
		path_points.append([x_dist, y_dist])
		path_points.append([x_dist+straight_part_length, y_dist])
		path_points.append([x_dist+straight_part_length, 0])

		return path_points

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

	def convert_path_points_to_world(self, path_points):
		realworld_poses = []
		try:
			model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			coords = model('robot_description', '')
			position = (coords.pose.position.x, coords.pose.position.y)

			orientation = coords.pose.orientation
			quaternions = [orientation.x, orientation.y, orientation.z, orientation.w]
			euler = efq(quaternions)

			target_frame = 'pioneer/map'
			current_frame = 'pioneer/base_link'

			tf_buff = tf2_ros.Buffer()
			tf_listener = tf2_ros.TransformListener(tf_buff)
			listener = tfi.TransformListener()
			listener.waitForTransform(target_frame, current_frame,rospy.Time(), rospy.Duration(4.0))
			tf = tf_buff.lookup_transform(target_frame, current_frame, rospy.Time())

			for pose in path_points:
				pose_msg = PoseStamped()
				pose_msg.header.frame_id = current_frame

				pose_msg.pose.position.x = pose[0]*np.cos(euler[2]) + pose[1]*np.sin(euler[2])
				pose_msg.pose.position.y = pose[0]*np.sin(euler[2]) - pose[1]*np.cos(euler[2])

				# pose_msg.pose.position.x = pose[0]
				# pose_msg.pose.position.y = pose[1]
				# pose_msg.pose.orientation.x = quaternions[0]
				# pose_msg.pose.orientation.y = quaternions[1]
				# pose_msg.pose.orientation.z = quaternions[2]
				# pose_msg.pose.orientation.w = quaternions[3]
				realworld_poses.append(pose_msg)
				# realworld_poses.append(tf2_geometry_msgs.do_transform_pose(pose_msg, tf))

			# print realworld_poses

		except rospy.ServiceException as exc:
			rospy.loginfo('Couldn\'t get model state: {}'.format(exc))
			
		return realworld_poses, position

	def convert_curve_to_world(self, path):
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

			return path

			# target_frame = '/pioneer/map'
			# current_frame = '/pioneer/base_link'

			# tf_buff = tf2_ros.Buffer()
			# tf_listener = tf2_ros.TransformListener(tf_buff)
			# tf = tf_buff.lookup_transform(target_frame, current_frame, rospy.Time(0))
			# realworld_path = tf2_geometry_msgs.do_transform_pose(path, tf)

			# print realworld_path

		except rospy.ServiceException as exc:
			rospy.loginfo('Couldn\'t get model state: {}'.format(exc))

		return path

	def plot_avoidance_path(self, p, pos=None):
		self.aaa = False
		if self.aaa == False:
			clf()
			if not pos==None:
				plt.scatter(pos[0], pos[1], c='r')
			for sig_p in p:
				plt.scatter(sig_p.pose.position.x, sig_p.pose.position.y)
			plt.show()
			self.aaa = True

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass