#!/usr/bin/env python

import tf
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import GetModelState


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

		while not rospy.is_shutdown():
			try:
				model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
				coords = model('robot_description', '')
				position = (coords.pose.position.x, coords.pose.position.y)
				print coords
				print velocity
				distance = self.get_distance(position)
				if self.start_distance == None:
					self.start_distance = distance

				if distance <= self.start_distance/2:
					if self.decrease_in_speed == None:
						velocity = (0, 0) if np.abs(coords.twist.linear.x) <= 0.0001 and np.abs(coords.twist.linear.y) <=  0.0001 \
								  else (coords.twist.linear.x, coords.twist.linear.y)
						self.decrease_in_speed = get_velocity_magnitude(velocity) - self.end_speed
					else:
						pub_vel = Twist()
						pub_vel.linear.



			except rospy.ServiceException as exc:
				rospy.loginfo('Couldn\'t get model state: {}'.format(exc))
				break

			self.rate.sleep()

	def get_distance(self, position):
		return np.sqrt((position[0]-goal[0])**2 + (position[1]-goal[1])**2)

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