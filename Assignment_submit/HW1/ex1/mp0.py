#!/usr/bin/env python3

#==============================================================================
# File name          : mp0.py                                                                 
# Description        : MP0 for CS588                                                                                                                        
# Usage              : rosrun mp0 mp0.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt

class Node():

	def __init__(self):

		self.rate = rospy.Rate(10)

		self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size = 1) # '/pacmod/as_rx/steer_cmd' is a topic

		# A ROS topic has a unique name, which is used by nodes to identify the topic they want to publish to or subscribe from. 
		# Messages published to a topic are delivered to all nodes that have subscribed to that topic. 
		#  Subscribers receive messages in the order in which they were published.

		self.steer_cmd = PositionWithSpeed()
		self.steer_cmd.angular_position = 0.75 # radians, -: clockwise, +: counter-clockwise
		self.steer_cmd.angular_velocity_limit = 2.0 # radians / second

		self.pacmod_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 2)
		self.Pacmod_Cmd = PacmodCmd()
		self.Pacmod_Cmd.enable = True

	def run(self):

		num_list = [2, 2, 2, 0, 0, 0, 2, 2, 2, 1]
		while not rospy.is_shutdown():
			self.steer_pub.header = Header()
			self.steer_pub.publish(self.steer_cmd)
			for num in num_list:
				self.Pacmod_Cmd.ui16_cmd=num
				self.pacmod_pub.publish(self.Pacmod_Cmd)

				self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	node.run()
	rospy.spin()