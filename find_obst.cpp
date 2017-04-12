//
// Created by anna on 2/5/17.
// Edited by Chong Swee 4/2/17

/*!
 Created: Feb 5, 2017
 Finds obstacles, converts their location to (x,y) coords, and stores as object
 Identifies obstacles, find a clear path for the robot to go through
 Publishes Twist (cmd_vel) of clearest path
 Publish boolean (whether there is an obstacle within lesss than THRESHOLD_STOP)
*/


#include <string>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

#define THRESHOLD 0.3 // THRESHOLD FOR CLEAR
#define VELOCITY 1
#define ROBO_WIDTH 0.5 // in meters

sensor_msgs::LaserScan scan;
sensor_msgs::LaserScan filtered_scan;

ros::Publisher obstacle_flag;
ros::Publisher lidar_vel;

geometry_msgs::Twist twist;
std_msgs::Bool flag;

void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
;

void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
    // set default speed
    twist.linear.x = VELOCITY;
    twist.angular.z = 0; // straight

    scan = lidar_scan;
    int number_of_ranges = lidar_scan.ranges.size();

    //always turn right if middle blocked (< THRESHOLD )
    if(scan.ranges[number_of_ranges/2]<THRESHOLD) {
        // turn to the left
        twist.angular.z = 3.142 / 12;
        flag.data =  0;
        obstacle_flag.publish(flag);
        lidar_vel.publish(twist);
        ROS_INFO("Obstacle detected at centre, moving left");
    }
    else{
        ROS_INFO("MOVING STRAIGHT");
        flag.data = 0;
        obstacle_flag.publish(flag);
        lidar_vel.publish(twist);

    }

    return;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "midbrain");

    ros::NodeHandle n;

    // create topics to publish to
    obstacle_flag = n.advertise<std_msgs::Bool>("obst/flag",1000);
    lidar_vel =n.advertise<geometry_msgs::Twist>("obst/cmd_vel", 1000);

    // subsribe to topics
    ros::Subscriber sub_lidar = n.subscribe("scan",1000,controlSpeed);

    ros::spin();

    return 0;
}