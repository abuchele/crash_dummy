#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "std_msgs/String.h"


ros::Publisher cmd_vel_pub;

ros::Time rstart;
ros::Time sstart;

ros::Duration cycle_time (10.0);
ros::Duration spin_time (10.0);
ros::Duration one_time (1.0);
int random_counter = 0;
bool spin = false;
int spin_counter = 0;
int zval;
int xval;
int zabs;

void one_random(){
    int z1;
    geometry_msgs::Twist msg;
    srand (time(NULL));
    z1 = rand() % 61;
    zval = z1 - 30;
    zabs = abs(zval);
    if (zabs < 10){
        xval = 12;
    }
    else if (zabs < 20) {
        xval = 18;
    }
    else {
        xval = 22;
    }
    msg.angular.z = zval;
    msg.linear.x = xval;
    cmd_vel_pub.publish(msg);
}

void one_spin(){
    geometry_msgs::Twist msg;
    msg.linear.x = 22;
    msg.angular.z = -30;

    cmd_vel_pub.publish(msg);
}


void timerCallback(const ros::TimerEvent& event){
    if (spin) {
        one_spin();
        spin_counter += 1;
        if (spin_counter > 9){
            spin_counter = 0;
            spin = false;
        }
    }
    else {
        one_random();
        random_counter += 1;
        if (random_counter > 9){
            random_counter = 0;
            spin = true;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "walk");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("rwk/cmd_vel", 1000);
    ros::Timer timer = n.createTimer(ros::Duration(1.0), (const ros::TimerCallback &) timerCallback);
    ros::spin();
    return 0;
}


