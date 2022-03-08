#pragma once //防止重复引用造成二义性，类似#ifndef

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#define PI 3.1415926

class GridCore
{

private:
    float grid_size = 0.1;
    float map_x0 = 0;
    float map_y0 = -10;
    float map_z0 = 0;
    int width_x = round(10 / grid_size);
    int width_y = round(20 / grid_size);

    ros::Subscriber sub_point_cloud_; //接受话题

    ros::Publisher pub_grid_, pub_2d_; //发布话题

    std_msgs::Header point_cloud_header_;

    void publish_cloud(const ros::Publisher &in_publisher,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                       const std_msgs::Header &in_header);

    void Point_2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    void grid_create(const pcl::PointCloud<pcl::PointXYZ>::Ptr in);
    void grid_main(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

public:
    GridCore(ros::NodeHandle &nh);
    ~GridCore();
    void Spin();
};