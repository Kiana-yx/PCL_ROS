#include "tim_base_core.h"

TimBaseCore::TimBaseCore(ros::NodeHandle &nh)
{
    sub_point_scan_ = nh.subscribe("/scan", 10, &TimBaseCore::LaserCallBack, this);
    sub_point_cloud_ = nh.subscribe("/cloud", 10, &TimBaseCore::LaserCallBack, this);

    pub_cylinder_ = nh.advertise<sensor_msgs::PointCloud2>("/cylinder", 10);
    
    //程序到达ros::spin()之前按照一系列规则，设定一系列话题订阅者
    //ros::spin()使订阅者们可以开始接受话题，进入回调函数
    ros::spin();
}

TimBaseCore::~TimBaseCore() {}

void TimBaseCore::Spin()
{
}



void TimBaseCore::LaserCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   unsigned long len = msg->ranges.size();
        std::vector<float> filtered_scan;
        for (int i = 0; i < len; ++i) {
            if(std::isnormal(msg->ranges[i])){
                filtered_scan.push_back(msg->ranges[i]);
            }
        }
        for (int j = 0; j < filtered_scan.size(); ++j) {
            ROS_INFO_STREAM(filtered_scan[j]);
        }
}