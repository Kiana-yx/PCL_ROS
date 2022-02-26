#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Person Info");
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "person_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;
    ROS_INFO("Subcribe Person Info------");
    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber point_sub = n.subscribe("/velodyne_points", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
