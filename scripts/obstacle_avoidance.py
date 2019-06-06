#!/usr/bin/env python

import tf as tfi
import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from numpy.linalg import norm
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Path, Odometry
from obstacle_detector.msg import Obstacles
from tf.transformations import euler_from_quaternion as efq
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.ops import nearest_points
from visualization_msgs.msg import Marker


class ObstacleAvoidance(object):

	def __init__(self):

		obstacles_topic = '/raw_obstacles'
		pub_topic = '/pioneer/path'
		path_topic = '/pioneer/curve_path'
		odom_topic = '/pioneer/odom'

		self.robot_pose = None
		self.global_path_received = False
		self.global_path = None
		self.global_path_line_string = None
		self.aaa = False
		self.robot_radius = 0.4
		self.obstacle_tolerance = 0.5
		self.path_forward = 100

		self.recognized_obstacles = []
		self.recognized_non_obstacles = []

		rospy.init_node('obstacle_avoidance', anonymous=True)
		rospy.Subscriber(obstacles_topic, Obstacles, self.avoidance_algorithm)
		rospy.Subscriber(odom_topic, Odometry, self.get_robot_pose)
		rospy.Subscriber(path_topic, Path, self.global_path_receiver)
		self.pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)
		self.pub_rviz = rospy.Publisher('/change_path', Marker, queue_size=10)

		rospy.spin()

	def get_robot_pose(self, data):
		self.robot_pose = data

	def global_path_receiver(self, data):
		self.global_path_received = True
		self.global_path = data.poses
		path_parts = []
		for part in self.global_path:
			path_parts.append((part.pose.position.x, part.pose.position.y))
		self.global_path_line_string = LineString(path_parts)
		self.log_info('Global Path Received')

	def avoidance_algorithm(self, data):
		# check if obstacle in path
		if self.global_path_received:
			obstacle_in_path, obstacle = self.is_obstacle_in_path2(data)
			if obstacle_in_path:
				is_safe = self.safety_check()
				if is_safe:
					avoidance_path = self.get_avoid_path_points(obstacle)
					# realworld_path, pos = self.convert_path_points_to_world(avoidance_path)
					realworld_path = avoidance_path
					self.plot_avoidance_path(realworld_path)

	# check if obstacle is in the way
	def is_obstacle_in_path(self, obstacles):
		obstacle_radius = None
		obstacle_distance = None
		obstacle_found = False
		obstacle = None
		len_glo = len(self.global_path)
		for circ in obstacles.circles:
			circ_xy = [circ.center.x, circ.center.y]
			circ_point = Point(circ.center.x, circ.center.y)
			recognized_object = False
			obstacle_added = False
			if not self.recognized_obstacles == []:
				for ob in self.recognized_obstacles:
					if self.get_distance(ob[:-1], circ_xy) < self.obstacle_tolerance:
						recognized_object = True
			if not self.recognized_non_obstacles == []:
				for ob in self.recognized_non_obstacles:
					if self.get_distance(ob[:-1], circ_xy) < self.obstacle_tolerance:
						recognized_object = True
			if not recognized_object:
				for index in range(0, len_glo - 2, 2):
					p1 = (self.global_path[index].pose.position.x, self.global_path[index].pose.position.y)
					p2 = (self.global_path[index+2].pose.position.x, self.global_path[index+2].pose.position.y)
					line_seg = LineString([p1, p2])
					point_on = nearest_points(line_seg, circ_point)[0]
					distance_to_circ = circ_point.distance(point_on)
					if distance_to_circ < circ.true_radius + self.robot_radius:
						self.log_info('Object at point x: ' + str(circ_xy[0]) + ', y: ' + str(circ_xy[1]) + ' is recognized as obstacle!')
						obstacle = [
							circ_xy[0],
							circ_xy[1],
							circ.true_radius,
							self.get_angle(p1, p2),
							distance_to_circ,
							point_on.coords[0][0],
							point_on.coords[0][1],
						]
						self.recognized_obstacles.append(obstacle)
						obstacle_radius = circ.true_radius
						robot_xy = [self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y]
						obstacle_distance = self.get_distance(circ_xy, robot_xy)
						obstacle_added = True
						obstacle_found = True
						break
				if not obstacle_added:
					self.log_warning('Object at point x: ' + str(circ_xy[0]) + ', y: ' + str(circ_xy[1]) + ' is not on path!')
					self.recognized_non_obstacles.append([
						circ_xy[0], 
						circ_xy[1],
						circ.true_radius
					])
		return obstacle_found, obstacle


	# check if obstacle is in the way
	def is_obstacle_in_path2(self, obstacles):
		obstacle_found = False
		obstacle = None
		len_glo = len(self.global_path)
		for circ in obstacles.circles:
			circ_xy = [circ.center.x, circ.center.y]
			circ_point = Point(circ.center.x, circ.center.y)
			recognized_object = False
			obstacle_added = False
			if not self.recognized_obstacles == []:
				for ob in self.recognized_obstacles:
					if self.get_distance(ob[:-1], circ_xy) < self.obstacle_tolerance:
						recognized_object = True
			if not self.recognized_non_obstacles == []:
				for ob in self.recognized_non_obstacles:
					if self.get_distance(ob[:-1], circ_xy) < self.obstacle_tolerance:
						recognized_object = True
			if not recognized_object:
				for index in range(0, len_glo - self.path_forward+1):
					p = []
					for part in range(0, self.path_forward):
						p.append((self.global_path[index+part].pose.position.x, self.global_path[index+part].pose.position.y))
					line_seg = LineString(p)
					point_on = nearest_points(line_seg, circ_point)[0]
					distance_to_circ = circ_point.distance(point_on)
					if distance_to_circ < circ.true_radius + self.robot_radius:
						self.log_info('Object at point x: ' + str(circ_xy[0]) + ', y: ' + str(circ_xy[1]) + ' is recognized as obstacle!')
						line_points = []
						path_point_index = index
						for k in range(0, self.path_forward-1):
							line = LineString([p[k], p[k+1]])
							if line.distance(point_on) < 1e-8:
								line_points.append(p[k])
								line_points.append(p[k+1])
								path_point_index += k

						obstacle = [
							circ_xy[0],
							circ_xy[1],
							circ.true_radius,
							self.get_angle(line_points[0], line_points[1]),
							distance_to_circ,
							point_on.coords[0][0],
							point_on.coords[0][1],
							path_point_index
						]
						self.recognized_obstacles.append(obstacle)
						robot_xy = [self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y]
						obstacle_added = True
						obstacle_found = True
						break
				if not obstacle_added:
					self.log_warning('Object at point x: ' + str(circ_xy[0]) + ', y: ' + str(circ_xy[1]) + ' is not on path!')
					self.recognized_non_obstacles.append([
						circ_xy[0], 
						circ_xy[1],
						circ.true_radius
					])
		return obstacle_found, obstacle

	# check if obstacle is in the way
	def is_obstacle_in_path3(self, obstacles):
		obstacle_found = False
		obstacle = None
		for circ in obstacles.circles:
			circ_xy = [circ.center.x, circ.center.y]
			circ_point = Point(circ.center.x, circ.center.y)
			recognized_object = False
			if not self.recognized_obstacles == []:
				for ob in self.recognized_obstacles:
					if self.get_distance(ob[:-1], circ_xy) < self.obstacle_tolerance:
						recognized_object = True
			if not self.recognized_non_obstacles == []:
				for ob in self.recognized_non_obstacles:
					if self.get_distance(ob[:-1], circ_xy) < self.obstacle_tolerance:
						recognized_object = True
			if not recognized_object:
				point_on = nearest_points(self.global_path_line_string, circ_point)[0]
				distance_to_circ = circ_point.distance(point_on)
				if distance_to_circ < circ.true_radius + self.robot_radius:
					self.log_info('Object at point x: ' + str(circ_xy[0]) + ', y: ' + str(circ_xy[1]) + ' is recognized as obstacle!')
					line_points = []
					path_point_index = None
					for k in range(0, self.path_forward-1):
						line = LineString([
							(self.global_path[k].pose.position.x, self.global_path[k].pose.position.y),
							(self.global_path[k+1].pose.position.x, self.global_path[k+1].pose.position.y)
						])
						if line.distance(point_on) < 1e-8:
							line_points.append((self.global_path[k].pose.position.x, self.global_path[k].pose.position.y))
							line_points.append((self.global_path[k+1].pose.position.x, self.global_path[k+1].pose.position.y))
							path_point_index = k
					obstacle = [
						circ_xy[0],
						circ_xy[1],
						circ.true_radius,
						self.get_angle(line_points[0], line_points[1]),
						distance_to_circ,
						point_on.coords[0][0],
						point_on.coords[0][1],
						path_point_index
					]
					self.recognized_obstacles.append(obstacle)
					obstacle_found = True
				else:
					self.log_warning('Object at point x: ' + str(circ_xy[0]) + ', y: ' + str(circ_xy[1]) + ' is not on path!')
					self.recognized_non_obstacles.append([
						circ_xy[0], 
						circ_xy[1],
						circ.true_radius
					])
		return obstacle_found, obstacle

	# check if is safe to avoid obstacle
	def safety_check(self):
		return True

	# make avoiding path points
	# def get_avoid_path_points(self, r, d):
	# 	path_points = []
	# 	y_dist = 2*r + self.robot_radius
	# 	x_dist = d - self.robot_radius
	# 	straight_part_length = 2*self.robot_radius + 2*r

	# 	path_points.append([x_dist, 0])
	# 	path_points.append([x_dist, y_dist])
	# 	path_points.append([x_dist+straight_part_length, y_dist])
	# 	path_points.append([x_dist+straight_part_length, 0])

	# 	return path_points

	# make avoiding path points
	def get_avoid_path_points(self, obstacle):
		ob_xy = obstacle[0:2]
		r = obstacle[2]
		ang = obstacle[3]
		dist = obstacle[4]
		point_on = obstacle[5:7]
		point_index = obstacle[7]

		d = self.robot_radius + r

		p1 = None
		p4 = None

		p1, index1 = self.search_point(point_on, point_index, d, -1)
		p4, index4 = self.search_point(point_on, point_index, d, 1)
		p0, _ = self.search_point(p1, index1, d/2, -1)
		p5, _ = self.search_point(p4, index4, d/2, 1)

		ang += np.pi/2
		dx = (1.1*d-dist)*np.cos(ang)
		dy = (1.1*d-dist)*np.sin(ang)
		p2 = [p1[0]+dx, p1[1]+dy]
		p3 = [p4[0]+dx, p4[1]+dy]

		mid_23 = [(p2[0]+p3[0])/2, (p2[1]+p3[1])/2]
		if self.get_distance(mid_23, ob_xy) < d:
			p2 = [p1[0]-dx, p1[1]-dy]
			p3 = [p4[0]-dx, p4[1]-dy]

		path_points = [p0, p1, p2, p3, p4, p5, ob_xy, point_on]

		print path_points
		print dist

		return path_points

	def search_point(self, point_on, search_index, d, inc):
		while True:
			search_index = search_index + inc
			search_xy = (self.global_path[search_index].pose.position.x, self.global_path[search_index].pose.position.y)
			dist_p = self.get_distance(point_on, search_xy)
			if dist_p > d:
				p = search_xy
				if dist_p > 1.5*d:
					p = LineString([point_on, search_xy]).interpolate(d).coords[0]
				break
		return p, search_index

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
		id = 0
		for sig_p in p:
			if id == 6:
				plt.scatter(sig_p[0], sig_p[1], c='r')
				id += 1
			elif id == 7:
				plt.scatter(sig_p[0], sig_p[1], c='g')
				self.publish_rviz(sig_p, id)
			else:
				plt.scatter(sig_p[0], sig_p[1])
				self.publish_rviz(sig_p, id)
				id += 1
		plt.axis('equal')
		plt.show()
	
	def publish_rviz(self, pose, id):
		marker = Marker()
		marker.header.frame_id = "pioneer/map"
		marker.header.stamp = rospy.Time.now()
		marker.ns = str(id)
		marker.action = marker.ADD
		marker.type = marker.CYLINDER
		marker.id = 0

		marker.scale.x = 0.3
		marker.scale.y = 0.3
		marker.scale.z = 1

		marker.color.a = 1.0
		marker.color.g = 0 
		marker.color.b = 0 
		marker.color.r = 1

		marker.pose.position.x = pose[0]
		marker.pose.position.y = pose[1]
		marker.pose.position.z = 0
		self.pub_rviz.publish(marker)

	def log_info(self, info):
		print '\033[94m' + '[OBSTACLE AVOIDANCE]: ' + '\033[92m' + str(info) + '\033[0m'

	def log_warning(self, info):
		print '\033[94m' + '[OBSTACLE AVOIDANCE]: ' + '\033[93m' + str(info) + '\033[0m'
	
	def get_distance(self, start, end):
		return np.sqrt((start[0]-end[0])**2 + (start[1]-end[1])**2)

	def get_angle(self, p1, p2):
		return np.arctan2(p2[1]-p1[1], p2[0]-p1[0])

	def distance_to_line(self, l1, l2, p):
		l1 = np.array(l1)
		l2 = np.array(l2)
		p = np.array(p)
		return norm(np.cross(l2-l1, l1-p))/norm(l2-l1)

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass