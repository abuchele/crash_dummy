#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8MultiArray, Int8
from geometry_msgs.msg import Twist
import numpy as np

ARRAY_SIZE = 11
INPUTS = ['wpt', 'obst']

class Midbrain_Arbiter(object):
	def __init__(self):
		rospy.init_node('midbrain_arbiter')

		self.vel_array = np.zeros([len(INPUTS), ARRAY_SIZE])
		self.turn_array = np.zeros([len(INPUTS), ARRAY_SIZE])

		rospy.Subscriber('/wpt/cmd_vel', Int8MultiArray, self.wpt_cmd_vel_cb)
		rospy.Subscriber('/obst/cmd_vel', Int8MultiArray, self.obst_cmd_vel_cb)
		rospy.Subscriber('/obst/avoid', Int8, self.update_flag) #whether or not we are avoiding an obstacle!

		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	def update_flag(self, value):
		self.flag = value.data

	def wpt_cmd_vel_cb(self, vels):
		if(self.flag==0): #not object avoidance
			#self.update_array(msg.data, INPUTS.index('wpt'))
			msg = Twist()
			msg.linear.x = int(vels.data[0])
			msg.angular.z = int(vels.data[1])
			self.cmd_vel_pub.publish(msg)
		else: #in the event of object avoidance
			pass

	def obst_cmd_vel_cb(self, msg):
		self.update_array(msg.data, INPUTS.index('obst'))

	def update_array(self, data, row):
		data = np.asarray(data).reshape\
			([2, ARRAY_SIZE])
		self.vel_array[row] = data[0]
		self.turn_array[row] = data[1]


	def run(self):
		vel_sum_array = np.zeros(ARRAY_SIZE)
		turn_sum_array = np.zeros(ARRAY_SIZE)
		#vel_sum_array = np.array([1., 2, 3, 4, 5, -5, 5, 5 ,5, 2, 2])
		#turn_sum_array = np.array([0., 0, 10, 1, -3, 0, 3, 2, 1, 1, 1])

		for i in range(len(INPUTS)):
			vel_sum_array += self.vel_array[i]
			turn_sum_array += self.turn_array[i]
		print(vel_sum_array)
		vel = vel_sum_array.argmax()
		print(vel)
		turn = turn_sum_array.argmax()
		print(turn)
		msg = Twist()
		if vel == 6:
			msg.linear.x = 20
		elif vel == 5:
			msg.linear.x = 0
		elif vel == 4:
			msg.linear.x = -10
		if turn == 6:
			msg.angular.z = -10
		elif turn == 5:
			msg.angular.z = 0
		elif turn == 4:
			msg.angular.z = 10
		#msg.linear.x = 2*vel/(ARRAY_SIZE-1)-1
		#msg.angular.z = 2*turn/(ARRAY_SIZE-1)-1
		#print msg.linear.x
		self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
	main = Midbrain_Arbiter()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		main.run()
		r.sleep()