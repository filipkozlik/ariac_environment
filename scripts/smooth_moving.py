#!/usr/bin/env python

import tf
import sys
import time
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion as efq


class SmoothMove(object):
	
	def __init__(self, goal, end_speed, max_speed):

		self.goal = goal
		self.end_speed = end_speed
		self.max_speed = max_speed
		self.start_distance = None
		self.decrease_in_speed = None
		self.delta_time = 0.1

		rospy.init_node('control_robot_node', anonymous=True)
		self.pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
		self.rate = rospy.Rate(1/self.delta_time)

		try:
			model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			coords = model('robot_description', '')
			
			orientation = coords.pose.orientation
			quaternions = [orientation.x, orientation.y, orientation.z, orientation.w]
			euler = efq(quaternions)
			position = (coords.pose.position.x + 2, coords.pose.position.y, euler[2])
		except rospy.ServiceException as exc:
			rospy.loginfo('Couldn\'t get model state: {}'.format(exc))

		print 'Goal: ' + str(position)
		self.move_to(position, 0, 0.05)

		# self.speed_list = [
		# 	self.speed_profile(0, 2, 0, 1),
		# 	self.speed_profile(2, 1, 1, 0.5),
		# 	self.speed_profile(3, 2, 0.5, 0.5),
		# 	self.speed_profile(5, 1, 0.5, 0)
		# ]

		# self.speeds = np.abs(list(self.speed_list[0][1]) + list(self.speed_list[1][1][1:]) +  
		# 					list(self.speed_list[2][1][1:]) + list(self.speed_list[3][1][1:]))

		# self.times = np.abs(list(self.speed_list[0][0]) + list(self.speed_list[1][0][1:]) +  
		# 					list(self.speed_list[2][0][1:]) + list(self.speed_list[3][0][1:]))

		# self.speed_list = [
		# 	self.speed_profile(0, 3, 0.01, 0.5),
		# 	self.speed_profile(3, 4, 0.5, 0.01)
		# ]

		# self.speeds = np.abs(list(self.speed_list[0][1]) + list(self.speed_list[1][1][1:]))
		# self.times = np.abs(list(self.speed_list[0][0]) + list(self.speed_list[1][0][1:]))

		# distance_calculated = self.calculate_distance(self.times, self.speeds)

		# self.speed_index = 0
		# self.speed_len = len(self.speeds)

		# msg = Twist()

		# while not rospy.is_shutdown():
		# 	try:
		# 		model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		# 		coords = model('robot_description', '')
		# 		position = (coords.pose.position.x, coords.pose.position.y)
				
		# 		orientation = coords.pose.orientation
		# 		quaternions = [orientation.x, orientation.y, orientation.z, orientation.w]
		# 		euler = efq(quaternions)

		# 		if self.speed_index == 0:
		# 			self.speed_index += 1
		# 			start_time = time.time()
		# 			start_pos = position
		# 			speed_x = self.speeds[0]
		# 			msg = Twist()
		# 			msg.linear.x = speed_x
		# 			self.pub.publish(msg)
		# 		else:
		# 			cur_time = time.time() - start_time
		# 			try:
		# 				index = map(lambda x: x>cur_time, self.times).index(True)
		# 			except:
		# 				index = None
		# 			if index == None:
		# 				end_time = time.time()
		# 				end_pos = position
		# 				print 'Distance travelled: ' + str(self.get_distance(start_pos, end_pos))
		# 				print 'Calculated distance: ' + str(distance_calculated)
		# 				msg.linear.x = 0.0
		# 				self.pub.publish(msg)
		# 				print 'Time elapsed: ' + str(end_time - start_time)
		# 				break
		# 			else:
		# 				speed_x = self.speeds[index]
		# 				print speed_x
		# 				msg.linear.x = speed_x
		# 				self.pub.publish(msg)

		# 	except rospy.ServiceException as exc:
		# 		rospy.loginfo('Couldn\'t get model state: {}'.format(exc))
		# 		break

		# 	self.rate.sleep()
	
	def speed_profile(self, start_time, execute_time, vel_start, vel_end):
		x = [c*execute_time for c in [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]]
		y = [0, 0.01, 0.06, 0.16, 0.31, 0.5, 0.69, 0.84, 0.94, 0.99, 1]
		if vel_end < vel_start:
			y = list(reversed(y))
			y = [c*np.abs(vel_end-vel_start)+vel_end for c in y]
		else:
			y = [c*np.abs(vel_end-vel_start)+vel_start for c in y]
		if vel_end == vel_start:
			p = np.polyfit(x, y, 0)
		else:
			p = np.polyfit(x, y, 5)
		xp = np.linspace(0, execute_time, int(execute_time/0.1))
		yp = np.abs(np.polyval(p, xp))
		return [xp + start_time, yp]

	def calculate_distance(self, times, speeds):
		dist = 0
		list_len = len(times) - 1
		for index in range(1, list_len):
			dist += (times[index+1] - times[index]) * speeds[index]
		return dist

	"""
	goal -- in form tuple (x, y, yaw)
	"""
	def move_to(self, goal, end_speed, error_allowed):
		current_position, current_velocity = self.get_current_position_and_velocity()
		print 'Current: ' + str(current_position)
		if np.abs(goal[2] - current_position[2]) < 0.05: # move in straight line
			dist = self.get_distance(current_position, goal)
			speed = self.get_velocity_magnitude(current_velocity)
			if speed > 0.01 and np.abs(speed-end_speed) < 0.01: # move without changing speed
				print 'Pass'
				pass
			else:
				print 'Do something'
				end_speed += 0.2
				curve_part1 = self.speed_profile(0, dist, speed, 0.5)
				curve_part2 = self.speed_profile(0, dist, 0.5, end_speed)
				dist_curve2 = self.get_distance(curve_part2[0], curve_part2[1])
				curve1_end = False
				curve1_len = len(curve_part1[0])
				curve2_len = len(curve_part2[0])
				print 'Curve1: ' + str(curve1_len)
				print 'Curve2: ' + str(curve2_len)
				curve1_index = 0
				curve2_index = 0
				cmd_vel_msg = Twist()
				while not rospy.is_shutdown():
					if curve1_index < curve1_len: # curve1
						speed_x = curve_part1[1][curve1_index]
						curve1_index += 1
					else:
						current_position, current_velocity = self.get_current_position_and_velocity()
						current_distance = self.get_distance(current_position, goal)
						if current_distance <= dist_curve2: # curve2
							if curve2_index < curve2_len:
								speed_x = curve_part2[1][curve2_index]
								curve2_index += 1
							else:
								if current_distance <= error_allowed:
									print 'End Position: ' + str(current_position)
									print 'Goal: ' + str(goal)
									print 'Curve2 distance: ' + str(dist_curve2)
									cmd_vel_msg.linear.x = 0.0
									self.pub.publish(cmd_vel_msg)
									break
								else:
									speed_x = end_speed
						else: # constant speed
							speed_x = 0.5
					cmd_vel_msg.linear.x = speed_x
					self.pub.publish(cmd_vel_msg)
		else: # move in curved line
			pass
			
		
	def get_current_position_and_velocity(self):
		try:
			model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			coords = model('robot_description', '')
			orientation = coords.pose.orientation
			quaternions = [orientation.x, orientation.y, orientation.z, orientation.w]
			euler = efq(quaternions)
			position = (coords.pose.position.x, coords.pose.position.y, euler[2])
			speed = (coords.twist.linear.x,coords.twist.linear.y)
		except rospy.ServiceException as exc:
			rospy.loginfo('Couldn\'t get model state: {}'.format(exc))
			position = (None, None, None)
			speed = (None, None)
		return position, speed

	def get_distance(self, start, end):
		return np.sqrt((start[0]-end[0])**2 + (start[1]-end[1])**2)

	def get_velocity_magnitude(self, velocity):
		return np.sqrt(velocity[0]**2 + velocity[1]**2)

if __name__ == '__main__':
    try:
    	goal = (float(sys.argv[1]), float(sys.argv[2]))
    	end_speed = float(sys.argv[3])
    	try:
    		max_speed = float(sys.argv[4])
        	SmoothMove(goal, end_speed, max_speed)
    	except:
        	SmoothMove(goal, end_speed, 1.0)
    except rospy.ROSInterruptException:
        pass