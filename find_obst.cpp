//
// Created by anna on 2/5/17.
//

/*!
 Created: Feb 5, 2017
 Finds obstacles, converts their location to (x,y) coords, and stores as object
*/




#include <string>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"


std_msgs::Int8MultiArray cmd_array;

std::vector<float> average_ranges;

sensor_msgs::LaserScan scan;
sensor_msgs::LaserScan filtered_scan;


// ros::Publisher pub_arb;
ros::Publisher pub_filtered_scan;
ros::Publisher pub_flag;
ros::Publisher pub_vel;

std_msgs::Int8 flag;


float threshold = 0.5;
int rolling_length = 5;

//key: { , , , , max backwards, stop, max forward, )

void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
;

void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
    // DEBUG
    //ROS_INFO("Received Scan");


    int obs_count=0;


    // Assign LIDAR scan to global
    scan = lidar_scan;
    sensor_msgs::LaserScan obst_found;
    long number_of_ranges = lidar_scan.ranges.size();

    for (int i=0; i< (number_of_ranges-1); i++){
        if ((scan.ranges[i] > 3)&&(scan.ranges[i+1]>3))
        { continue;}
        else {
            if (fabs(scan.ranges[i] - scan.ranges[i + 1]) > threshold) {
                obst_found.ranges[obs_count] = scan.ranges[i];
                //ROS_INFO("obsts: %f", (obst_found.ranges));
                obs_count = obs_count + 1;
                //ROS_INFO("angle: %f", (((scan.angle_min) + (i * scan.angle_increment))*57.3));
            }
        }
    }
    //ROS_INFO("num obst: %d", obs_count);


    //ROS_INFO("number of ranges: %d", lidar_scan.ranges.size());


    pub_vel.publish(cmd_array);
    pub_flag.publish(flag);
    cmd_array.data.clear();

    // DEBUG
    //ROS_INFO("Publishing Output");

}

void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
{
    //ROS_INFO("Received Scan");

    scan.ranges = lidar_scan.ranges;
    filtered_scan.ranges = lidar_scan.ranges;
    filtered_scan.header.frame_id = lidar_scan.header.frame_id;
    filtered_scan.angle_min = lidar_scan.angle_min;
    filtered_scan.angle_max = lidar_scan.angle_max;
    filtered_scan.angle_increment = lidar_scan.angle_increment;
    filtered_scan.range_max = lidar_scan.range_max;
    filtered_scan.range_min = lidar_scan.range_min;
    //std::vector<int> indices;


    pub_filtered_scan.publish(filtered_scan);

    pub_vel.publish(cmd_array);
    cmd_array.data.clear();

}

int main(int argc, char **argv)
{
    flag.data=0;

    ros::init(argc, argv, "midbrain");

    ros::NodeHandle n;


    pub_flag = n.advertise<std_msgs::Int8>("obst/avoid",1000);
    pub_vel =n.advertise<std_msgs::Int8MultiArray>("obst/cmd_vel", 1000);
    //pub_ang =n.advertise<std_msgs::Int8MultiArray>("obst/cmd_dir", 1000);
    pub_filtered_scan =n.advertise<sensor_msgs::LaserScan>("obst/filtered_scan", 1000);
    // ros::Publisher pub_arb =n.advertise<std_msgs::Int8MultiArray>("obst/arb", 1000);

    //ros::Subscriber sub_imu = n.subscribe("scan", 1000, controlSpeed);
    ros::Subscriber sub_lidar = n.subscribe("scan",1000,controlSpeed);



    ros::spin();

    return 0;
}