#!/usr/bin/env python

#created by Anna on 3/20/2017
#This arbiter subscribes to all the topics publishing their recommended routes, as well as the mission status, and publishes the mission status
#as well as the correct path.

import rospy
from std_msgs.msg import Int8MultiArray, Int8
from geometry_msgs.msg import Twist
import numpy as np


class arbiter(object):
    def __init__(self):
        rospy.init_node('arbiter')

        self.flag = 0
        self.can_vel_x = 0
        self.can_vel_z = 0
        self.rwk_vel_x = 0
        self.rwk_vel_z = 0
        self.obst_vel_x = 0
        self.obst_vel_z = 0
        self.miss_stat = -1
        self.msg = Twist()

        rospy.Subscriber('/can_vel', Twist, self.can_vel_cb)
        rospy.Subscriber('/rwk_vel', Twist, self.rwk_vel_cb)
        rospy.Subscriber('/obst_vel', Twist, self.obst_vel_cb)
        rospy.Subscriber('/obst/avoid', Int8, self.update_flag) #whether or not we are avoiding an obstacle!
        rospy.Subscriber('/miss_stat', Int8, self.update_status)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.miss_stat_pub = rospy.Publisher('/miss_stat', Int8, queue_size=1)

    def update_flag(self, value):
        self.flag = value.data

    def status_pub(self):
        self.msg = Twist()
        if self.miss_stat == 0:
            msg.linear.x = 0
            msg.angular.z = 0 #mission status = 0: STOP
        elif self.miss_stat == 1: #random walk
            if self.flag:
                msg.linear.x = self.obst_vel_x
                msg.angular.z = self.obst_vel_z
            else:
                msg.linear.x = self.rwk_vel_x
                msg.angular.z = self.rwk_vel_z
        elif self.miss_stat == 2: #drive to can
            msg.linear.x = self.can_vel_x
            msg.angular.z = self.can_vel_z
        elif self.miss_stat == 3: #pick up can
            msg.linear.x = 0
            msg.angular.z = 0
        elif self.miss_stat == -1: #we're just starting up, so wait
            msg.linear.x = 0
            msg.angular.z = 0
        else:
            pass
            #go home

    def can_vel_cb(self,value):
        self.can_vel_x = value.linear.x
        self.can_vel_z = value.angular.z

    def rwk_vel_cb(self,value):
        self.rwk_vel_x = value.linear.x
        self.rwk_vel_z = value.angular.z

    def obst_vel_cb(self,value):
        self.obst_vel_x = value.linear.x
        self.obst_vel_z = value.angular.z

    def update_status(self, value):
        self.miss_stat = value.data

    def run(self):
        self.cmd_vel_pub.publish(self.msg)
        self.miss_stat_pub.publish(self.miss_stat)

if __name__ == '__main__':
    main = arbiter()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        main.run()
        r.sleep()