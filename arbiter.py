#!/usr/bin/env python

#created by Anna on 3/20/2017
#This arbiter subscribes to all the topics publishing their recommended routes, as well as the mission status, and publishes the mission status
#as well as the correct path.

import rospy
from std_msgs.msg import Int8MultiArray, Int8, Bool
from geometry_msgs.msg import Twist
import numpy as np

temporary_workaround = True #if the robot should ignore obstacle detection after finding the can

class arbiter(object):
    def __init__(self):
        rospy.init_node('arbiter')

        #initialize all the values we will be using
        self.flag = 0
        self.can_vel_x = 0
        self.can_vel_z = 0
        self.rwk_vel_x = 0
        self.rwk_vel_z = 0
        self.obst_vel_x = 0
        self.obst_vel_z = 0
        self.gohome_vel_x = 0
        self.gohome_vel_z = 0
        self.miss_stat = 1
        self.msg = Twist()
        self.can_picked = False;
        self.e_stop = False;

        #create all the subscribers for the topics for cmd_vel
        rospy.Subscriber('/gohome/cmd_vel', Twist, self.gohome_vel_cb) #topic which tells us how to go home
        rospy.Subscriber('/img_rec/cmd_vel', Twist, self.can_vel_cb)   #where to go to find the can
        rospy.Subscriber('/rwk/cmd_vel', Twist, self.rwk_vel_cb)       #how to random walk
        rospy.Subscriber('/obst/cmd_vel', Twist, self.obst_vel_cb)     #how to turn to avoid obstacles

        #create subscribers for the flags
        rospy.Subscriber('e_stop', Bool, self.e_stop_cb)           #whether e-stop is pressed
        rospy.Subscriber('/obst/flag', Bool, self.update_flag)         #whether we are avoiding an obstacle
        rospy.Subscriber('/can_picked', Bool, self.update_status)      #whether the can has been picked up yet

        #create subscribers for the mission status
        rospy.Subscriber('img_rec/miss_stat', Int8, self.update_status) #mission status published by img_rec
        rospy.Subscriber('/miss_stat', Int8, self.update_status)        #mission status published by arbiter

        #create publishers for cmd_vel (speed arduino will tell motors to go) and mission status (where we are in the mission)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)     #publish cmd_vel
        self.miss_stat_pub = rospy.Publisher('/miss_stat', Int8, queue_size=1)  #publish mission status

    def update_flag(self, value):
        self.flag = value.data  #update self value for the flag (whether we are avoiding obstacle)

    def status_pub(self):
        if self.miss_stat == 0: #if we are stopped
            self.msg.linear.x = 0
            self.msg.angular.z = 0 #mission status = 0: STOP
        elif self.miss_stat == 1: #random walk
            if self.flag:
                self.msg.linear.x = self.obst_vel_x
                self.msg.angular.z = self.obst_vel_z
            else:
                self.msg.linear.x = self.rwk_vel_x
                self.msg.angular.z = self.rwk_vel_z
        elif self.miss_stat == 2: #drive to can
            if self.flag:
                self.msg.linear.x = self.obst_vel_x
                self.msg.angular.z = self.obst_vel_z
            else:
                self.msg.linear.x = self.can_vel_x
                self.msg.angular.z = self.can_vel_z
        elif self.miss_stat == 3: #pick up can
            self.msg.linear.x = 0
            self.msg.angular.z = 0
            if self.can_picked == True:
                self.miss_stat = 4
        elif self.miss_stat == -1: #we're just starting up, so wait
            self.msg.linear.x = 0
            self.msg.angular.z = 0
            self.miss_stat = 1
        elif self.miss_stat == 4: #drive home! This functionality does not exist yet, but we are hopeful.
<<<<<<< HEAD
            #self.msg.linear.x = 0  
            #self.msg.angular.z = 0 

            #if this functionality exists, uncomment the next two lines, and comment out the previous two.
            if self.flag:
                self.msg.linear.x = self.obst_vel_x
                self.msg.angular.z = self.obst_vel_z
            else:
                self.msg.linear.x = self.gohome_vel_x
                self.msg.angular.z = self.gohome_vel_z 
=======
            self.msg.linear.x = self.gohome_vel_x
            self.msg.angular.z = self.gohome_vel_z
>>>>>>> 096c4f53e4053316b48b3f4f59e5ef670afe3e27
        else:
            pass
            #we shouldn't be here!

    #update self values for the flags, mission status, and cmd_vels specific to each program
    def e_stop_cb(self,value):
        self.e_stop = value.data

    def can_vel_cb(self,value):
        self.can_vel_x = value.linear.x
        self.can_vel_z = value.angular.z

    def gohome_vel_cb(self,value):
        self.gohome_vel_x = value.linear.x
        self.gohome_vel_z = value.angular.z

    def rwk_vel_cb(self,value):
        self.rwk_vel_x = value.linear.x
        self.rwk_vel_z = value.angular.z

    def obst_vel_cb(self,value):
        self.obst_vel_x = value.linear.x
        self.obst_vel_z = value.angular.z

    def update_status(self, value):
        self.miss_stat = value.data
        if self.miss_stat == 5:  #miss_stat 5 means restart
            self.flag = 0
            self.can_vel_x = 0
            self.can_vel_z = 0
            self.rwk_vel_x = 0
            self.rwk_vel_z = 0
            self.obst_vel_x = 0
            self.obst_vel_z = 0
            self.gohome_vel_x = 0
            self.gohome_vel_z = 0
            self.miss_stat = 1
            self.msg = Twist()
            self.can_picked = False;

    #run everything
    def run(self):
        self.status_pub()
        self.cmd_vel_pub.publish(self.msg)
        self.miss_stat_pub.publish(self.miss_stat)

if __name__ == '__main__':
    main = arbiter()
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        main.run()
        r.sleep()
