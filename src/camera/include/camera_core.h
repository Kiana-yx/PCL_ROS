#pragma once //防止重复引用造成二义性，类似#ifndef

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core.hpp"

class CameraCore
{

private:
    ros::Publisher pub_camera_; //发布话题
    
public:
    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;
    
    CameraCore(ros::NodeHandle &nh);
    ~CameraCore();
    void Spin();
};