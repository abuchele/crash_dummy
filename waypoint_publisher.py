#!/usr/bin/env python
'''
waypoint_publisher.py
---------------------

The purpose of this node is to read a course out
waypoint-by-waypoint and publish each to the
`/waypoint` topic. When the robot gets sufficiently
close to a waypoint (as read from the GPS), it will
publish the next waypoint to `/waypoint`.

Note: currently, the robot will never quite 'finish'
the course, but rather it will continue to try to get
to the waypoint.

'''

import rospy
import rospkg
import json
import os
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import NavSatFix

rospack = rospkg.RosPack()

# -- Config
COURSE = 'SAFE'

COURSEFILE = rospack.get_path('robo_team_charlie')+'/courses.json'

WAYPOINT_RADIUS = 2; # Waypoint radius in meters
DEBUG = True

LAT_TO_M = 111078.95354277734;
LONG_TO_M = 82469.1107701757;

# -- Definitions
class WptPublisher:
	def __init__(self, coursename):
		with open(COURSEFILE, 'r') as coursefile:
			self.courses = json.load(coursefile);

		self.pub = rospy.Publisher(
			'/waypoint',
			Float64MultiArray,
			queue_size=10
		)

		if DEBUG:
			self.chatter = rospy.Publisher(
				'/chatter',
				String,
				queue_size=10
			)

		self.sub = rospy.Subscriber(
			'/fix',
			NavSatFix,
			self.checkDistance
		)

		rospy.init_node(
			'waypoint_publisher',
			anonymous=True
		)

		self.setCourse(coursename)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			rate.sleep()

	def checkDistance(self, gpsPosition):
		delta_lat = (self.waypoint[0] - gpsPosition.latitude) * LAT_TO_M
		delta_long = (self.waypoint[1] - gpsPosition.longitude) * LONG_TO_M
		distance = (delta_lat**2 + delta_long**2) ** .5

		if DEBUG:
			self.chatter.publish(str(distance))

		if distance < WAYPOINT_RADIUS:
			nextWpt()

	def setCourse(self, coursename):
		self.course = self.courses[coursename]
		self.index = 0
		self.nextWpt()

	def nextWpt(self):
		if self.index < len(self.course):
			self.pubWpt(self.index)
			self.index += 1

	def pubWpt(self, wpt):
		self.waypoint = self.course[wpt][1:3]
		msg = Float64MultiArray(data=self.waypoint)
		self.pub.publish(msg)

# -- Running Code
if __name__ == "__main__":
	try:
		w = WptPublisher(COURSE)
	except rospy.ROSInterruptException:
		pass