#!/usr/bin/env python

#created by Anna on 3/20/2017
#edited by  CS on 4/11/2017
# <<taken from can_size_csedit.py >>
# Subscibes to:
#     (Bool)    /can/flag           whether can is present
#     (Float)   /can/distance       distance of can in cm
#     (Bool)    /can/position_flag  whether can is at the correct position to be picked up
#                                   1. if abs(angle) <5
#                                   2. distance ideal (16cm-20cm)
#     (Float)   /can/angle          angle from the centre (-ve to the left 0 in centre, +ve to the right) in degrees
#     (Bool)    /obst/flag          whether obstacle is present
#                          !! if obstacle is present, turn left until flag == 0
#    (Twist)    /rwk_vel            Random walk vel info
# !!!!!NOTE THAT COKE CAN MIGHT BE MISTAKEN AS AN OBSTACLE!!!!!
#   Based on information from subscribed topics, publishes :
#    (Twist)    /cmd_vel            linear and angular velocicty information for the crash dummy
#This arbiter subscribes to all the topics publishing their recommended routes, as well as the mission status, and publishes the mission status
#as well as the correct path.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import numpy as np

class arbiter(object):
    def __init__(self):
        rospy.init_node('arbiter')

        self.can_flag = 0
        self.can_distance =0
        self.can_position_flag = 0
        self.can_angle =0
        self.obst_flag = 0
        self.rwk_vel_x = 0
        self.rwk_vel_z = 0
        self.msg = Twist()

        rospy.Subscriber('/can/flag', Bool , self.can_flag_cb) #can if twist.angular.z > 0 then present
        rospy.Subscriber('/can/distance', Float64, self.can_distance_cb) #random walk
        rospy.Subscriber('/can/position_flag', Bool, self.can_position_flag_cb) # to avoid obstacle
        rospy.Subscriber('/can/angle', Float64, self.can_angle_cb) #whether or not we are avoiding an obstacle!
        rospy.Subscriber('/obst/flag', Bool, self.obst_flag_cb) #whether or not we are avoiding an obstacle!
        rospy.Subscriber('/rwk_vel', Twist, self.rwk_vel_cb) #whether or not we are avoiding an obstacle!

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def update_flag(self, value):
        self.flag = value.data

    # Based on subscribed messages, decide on cmd_vel to publilsh
    def status_pub(self):
        print("obst flag", self.obst_flag)
        if self.can_position_flag == 1: # if can is at the right position, stop
            self.msg.linear.x = 0
            self.msg.angular.z = 0
            print("1")
        elif self.can_flag == 1: #if can is found, but not in the right position
            print("2",self.can_angle)

            if (abs(self.can_angle) > 6):
                if self.can_angle > 0: #turn right
                    print("turn right")
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 1.0
                else:                 #turn left
                    print("turn lefts")
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = -1.0
            else: # if within angle threshold, move forward
                self.msg.linear.x = 1.0
                self.msg.angular.z = 0.0
        elif self.obst_flag == 1:   #turn lef    t
            print("3")
            self.msg.linear.x = -1
            self.msg.angular.z = 0
        else:    #random walk
            print("4")
            self.msg.linear.x = self.rwk_vel_x
            self.msg.angular.z = self.rwk_vel_z

    def can_flag_cb(self,value):
        self.can_flag = value.data

    def rwk_vel_cb(self,value):
        self.rwk_vel_x = value.linear.x
        self.rwk_vel_z = value.angular.z

    def can_distance_cb(self,value):
        self.can_distance = value.data

    def can_position_flag_cb(self,value):
        self.can_position_flag = value.data

    def can_angle_cb(self,value):
        self.can_angle = value.data

    def obst_flag_cb(self,value):
        self.obst_flag = value.data

    def run(self):
        self.cmd_vel_pub.publish(self.msg)
        print("published!")

if __name__ == '__main__':
    main = arbiter()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        main.status_pub()
        main.run()
        r.sleep()
