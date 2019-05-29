#!/usr/bin/env python

import tf
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped


class TransformGoal(object):

	def __init__(self, goal):

		self.goal = goal
		self.trans_from = rospy.get_param("~map_frame")
		self.trans_to = rospy.get_param("~robot_frame")

		self.tf_listener = tf.TransformListener()
		self.tf_listener.waitForTransform(
			self.trans_from, 
			self.trans_to,
			rospy.Time(0),
			rospy.Duration(4.0)
		)

		self.goal_message = PoseStamped()
		self.goal_message



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

	def constructe_message(self):
		message = PoseStamped()
		message.header.frame_id = self.tra

if __name__ == '__main__':
    try:
    	goal = (float(sys.argv[1]), float(sys.argv[2]))
        TransformGoal(goal)
    except rospy.ROSInterruptException:
        pass