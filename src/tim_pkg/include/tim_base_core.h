#pragma once //防止重复引用造成二义性，类似#ifndef

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.1415926

class TimBaseCore
{

private:
    ros::Subscriber sub_point_cloud_; //接受话题
    ros::Subscriber sub_point_scan_; //接受话题

    ros::Publisher pub_cylinder_; //发布话题

    void point_main(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

public:
    TimBaseCore(ros::NodeHandle &nh);
    ~TimBaseCore();
    void Spin();
};