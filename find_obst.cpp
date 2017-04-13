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
#define VELOCITY 25
#define ROBO_WIDTH 0.5 // in meters

sensor_msgs::LaserScan scan;
sensor_msgs::LaserScan filtered_scan;

ros::Publisher obstacle_flag;
ros::Publisher lidar_vel;

geometry_msgs::Twist twist;
std_msgs::Bool flag;
float smallest;
float angle_val;

void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
;



void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
    float forward_distance = 100;
    // set default speed
    twist.linear.x = VELOCITY;
    twist.angular.z = 0; // straight

    scan = lidar_scan;
    int number_of_ranges = (int) lidar_scan.ranges.size();

    filtered_scan.ranges = lidar_scan.ranges;
    filtered_scan.header.frame_id = lidar_scan.header.frame_id;
    filtered_scan.angle_min = lidar_scan.angle_min;
    filtered_scan.angle_max = lidar_scan.angle_max;
    filtered_scan.angle_increment = lidar_scan.angle_increment;
    filtered_scan.range_max = lidar_scan.range_max;
    filtered_scan.range_min = lidar_scan.range_min;

    // Remove junk values from scan data (0.0 is out of range or too close to be accurate)
    for(int i=0; i < sizeof(scan.ranges) / sizeof(scan.ranges[0]); i++)
    {
        if(filtered_scan.ranges[i] < 0.1 || filtered_scan.ranges[i] > 3.5)
        {
            filtered_scan.ranges[i] = 0.0;
        }
    }
    // Calculate output array using some portion of scan -> set "forward distance" to the smallest value -> the closest object.
    for (int i = number_of_ranges/3; i<2*number_of_ranges/3;i++)
    {
        if ((forward_distance > filtered_scan.ranges[i]) && (filtered_scan.ranges[i] >= 0.1)) {
            forward_distance = filtered_scan.ranges[i];
            smallest = i;
        }
    }

    if (forward_distance < .2)
    {
        // Move backward
        flag.data=1; //avoiding obstacles
        twist.angular.z = 0; //go straight backwards
        twist.linear.x = -20; //back up, slowly
    }

    else if (forward_distance < 0.7)
    {
        flag.data = 1;
        angle_val = (smallest)/ (number_of_ranges);
        if (angle_val < 0.5) {
            //turn left
            twist.angular.z = -50;
        }
        else if (angle_val >= 0.5){
            //turn right
            twist.angular.z = 50;
        }

    }
    else {
        // Move forward
        flag.data=0; //not avoiding obstacles
        twist.angular.z = 0; //go straight forward
        twist.linear.x = 30; //drive forward
        //ROS_INFO_STREAM(cmd_array);
    }
    obstacle_flag.publish(flag);
    lidar_vel.publish(twist);

    forward_distance = 100.0;
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
