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

int counterRand = 0;
int counterSpin = 0;

while (true){

while (counterRand < (timeS*2))
{
	int z_rand, z1;


	geometry_msgs::Twist msg;

	srand (time(NULL));
	z1 = rand() % 201;



	msg.linear.x = 20;
	msg.angular.z = z1 -100;

	chatter_pub.publish(msg);

	loop_rate.sleep();

	ros::Duration run_time = ros::Time::now() - begin;
	counterRand = counterRand + 1;
}

while (counterSpin < timeS){
	geometry_msgs::Twist msg;
	msg.linear.x = 20;
	msg.angular.z = -100;
	chatter_pub.publish(msg);
	loop_rate.sleep();
	counterSpin = counterSpin +1;
}
counterRand = 0;
counterSpin = 0;


ros::spinOnce(); 
}
return 0;
}


