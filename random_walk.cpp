#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "std_msgs/String.h"


int main(int argc, char **argv)
{

ros::init(argc, argv, "walk");

ros::NodeHandle n;

ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("rwk/cmd_vel", 1000);

ros::Rate loop_rate(10);

ros::Time begin = ros::Time::now();
ros::Duration run_time;

int timeS = 20;

while (ros::ok() && run_time.toSec() < timeS)
{
	int z_rand, z1;


	geometry_msgs::Twist msg;

	srand (time(NULL));
	z1 = rand() % 201;



	msg.linear.x = 30;
	msg.angular.z = z1 -100;

	chatter_pub.publish(msg);

	ros::spinOnce(); 
	loop_rate.sleep();

	ros::Duration run_time = ros::Time::now() - begin;
}
return 0;
}


