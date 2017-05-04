import rospy
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class returner:

    def __init__(self):

        rospy.init_node("returner")
        self.pub = rospy.publisher("gohome/cmd_vel", Twist)
        #rospy.Subscriber("goHome", tuple, self.encoder_callback)
        #rospy.Subscriber("lidar_vel", twist, self.get_back)
        self.leftChange = 0
        self.rightChange = 0
        self.angle = np.pi/2.0;
        self.position = [0,0] #x, y from starting point in cm
        self.twist = 0
        self.is_clear = True
        self.obstacle_pos = False

        self.WHEEL_DIAMETER = 10 #cm
        self.ROBOT_WIDTH = 25 #cm, distance between wheels
        #Conversion factor for one full rotation of pot reading, to length.
        self.P2L = (self.WHEEL_DIAMETER*np.pi) / 100  #circumference/fullrotationreading..
        self.twist = Twist()



    def encoder_callback(self, data):
            raw = data.data
            self.leftChange = raw[0]
            self.rightChange = raw[1]

    def update_position(self):
        right_length = self.right_change * self.P2L
        left_length = self.left_change * self.P2L
        angleChange = (right_length - left_length)/ (ROBOT_WIDTH)
        dcenter = (left_length - right_length) / 2.0

        #update position data
        self.position[0] = dcenter*np.cos(self.angle)
        self.position[1] = dcenter*np.sin(self.angle)
        self.angle = self.angle + angleChange

#http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
    def get_back(self):
        #Do a thing where with the lidar, have a range of possible angles (0-180).
        #Arbiter will will make it so that prioritizing obstacle avoidance,
        #so of those angles, cannot choose the ones that would result in a collision.
        #Otherwise if no obstacles, can achieve prime directive.

        #Alternatively, if have a map, then can set temporary waypoints
        #this would help a lot.
        ideal_angle = np.arctan(self.position[1] / self.position[0]) + np.pi/2.0

        # if self.is_clear == False: #To be implemented
        #     #wall_follow #Basically turn around to one angle some and move forward.
        #     if self.obstacle_pos = right or center:
        #         self.twist.linear.x = 10
        #         self.twist.angular.z = -50
        #     elif self.obstacle_pos = left:
        #         self.twist.linear.x = 10
        #         self.twist.angular.z = 50

        if (self.angle - ideal_angle) < ((-10.0/360.0)/(2*np.pi)):
            self.twist.linear.x = 10
            self.twist.angular.z = 50 #right turn.
        elif (self.angle - ideal_angle) > (10.0/360.0)/(2*np.pi):
            self.twist.linear.x = 10
            self.twist.angular.z = -50 #Left turn
        else:
            self.twist.linear.x = 20 #forward
            self.twist.angular.z = 0

        pub.publish(twist)
