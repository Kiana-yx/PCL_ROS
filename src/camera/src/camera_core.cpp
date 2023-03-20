#include "camera_core.h"

CameraCore::CameraCore(ros::NodeHandle &nh)
{
    // pub_camera_ = nh.advertise<sensor_msgs::PointCloud2>("/camera", 10);

    //程序到达ros::spin()之前按照一系列规则，设定一系列话题订阅者
    //ros::spin()使订阅者们可以开始接受话题，进入回调函数
    ros::spin();
}

CameraCore::~CameraCore() {}

void CameraCore::Spin()
{
}