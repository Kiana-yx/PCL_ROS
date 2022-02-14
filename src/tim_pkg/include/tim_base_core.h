#pragma once //防止重复引用造成二义性，类似#ifndef

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#define PI 3.1415926

#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

#define RAD2DEG(x) ((x)*180./M_PI)
#define max_scan_count 271

struct TIM_Point
{
    float value;
    size_t index;
    float x;
    float y;
};

// 排序的规则,从小到大进行排序
struct by_value
{
    bool operator()(TIM_Point const &left, TIM_Point const &right)
    {
        return left.value < right.value;
    }
};

class TimBaseCore
{

private:
    ros::Subscriber sub_point_cloud_; //接受话题
    ros::Subscriber sub_point_scan_; //接受话题

    ros::Publisher pub_cylinder_; //发布话题

    void LaserScanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void LaserCloudCallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

public:
    TimBaseCore(ros::NodeHandle &nh);
    ~TimBaseCore();
    void Spin();
};