#include "tim_base_core.h"

TimBaseCore::TimBaseCore(ros::NodeHandle &nh)
{
    sub_point_scan_ = nh.subscribe("/scan", 10, &TimBaseCore::LaserScanCallBack, this);
    sub_point_cloud_ = nh.subscribe("/cloud", 10, &TimBaseCore::LaserCloudCallBack, this);

    pub_cylinder_ = nh.advertise<sensor_msgs::PointCloud2>("/cylinder", 10);

    //程序到达ros::spin()之前按照一系列规则，设定一系列话题订阅者
    //ros::spin()使订阅者们可以开始接受话题，进入回调函数
    ros::spin();
}

TimBaseCore::~TimBaseCore() {}

void TimBaseCore::Spin()
{
}

void TimBaseCore::LaserScanCallBack(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::vector<TIM_Point> scan_smoothness_(max_scan_count); // 存储每个点的曲率与索引
    float *scan_curvature_ = new float[max_scan_count];      // 存储每个点的曲率

    int count = 0;                  // 有效点的索引
    std::map<int, int> map_index;   // 有效点的索引 对应的 scan实际的索引
    float new_scan[max_scan_count]; // 存储scan数据的距离值

    int len = scan->ranges.size();
    for (int i = 0; i < len; ++i)
    {
        if (std::isfinite(scan->ranges[i])) //确定给定的浮点数 arg 是否拥有有限值，即它是正规、非正规或零，但不是无穷大或 NaN
        {
            continue;
        }

        // 这点在原始数据中的索引为i，在new_scan中的索引为count
        map_index[count] = i;
        // new_scan中保存了有效点的距离值
        new_scan[count] = scan->ranges[i];
        count++;
    }

    ROS_INFO("I heard a laser scan %s:", scan->header.frame_id.c_str());
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max)); //这个坐标似乎与与认知上有旋转？
}

void TimBaseCore::LaserCloudCallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud)
{
}