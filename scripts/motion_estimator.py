#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry

class integrator:
	def __init__(self):

		self.odom_sub = rospy.Subscriber('/Odom', Odometry, self.odomCallback)
		self.predicted_x = 0
		self.predicted_y = 0
		self.predicted_z = 0
		self.firstTime = True
	def odomCallback(self, data):
		if !self.firstRun:
			vel_x = data.twist.twist.linear.x
			vel_y = data.twist.twist.linear.y
			vel_z = data.twist.twist.linear.z

			self.predicted_x = self.predicted_x + vel_x*dt;
			self.predicted_y = self.predicted_y + vel_y*dt;
			self.predicted_z = self.predicted_z + vel_z*dt; 
		
if __name__ == '__main__':
	rospy.init_node('motion_estimator', anonymous=True)
	odometry = integrator()
	rospy.spin()