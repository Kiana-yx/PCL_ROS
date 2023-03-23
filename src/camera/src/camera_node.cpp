#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

bool is_calibrate = true;

using namespace cv;
using namespace std;

std_msgs::Header header;

ros::Publisher pub_camera_; // 发布话题

int main(int argc, char **argv)
{

    ros::init(argc, argv, "camera_node"); // 初始化节点，第三个参数 node_name
    std::cout << "camera_node init" << std::endl;
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 10);

    VideoCapture cap(0);

    if (!cap.isOpened())
    {
        ROS_INFO("Can not open camera!");
        return -1;
    }
    else
    {
        ROS_INFO("Camera opened success...");
    }
    ros::Rate loop_rate(15); // 设定15Hz

    // 根据相机参数选择一个分辨率
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cv::Mat frame;
    header.frame_id = "world";

    Mat frameCalibration;
    Mat view, rview, map1, map2;
    Size imageSize;
    if (is_calibrate)
    {
        Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = 1.2157e+03;
        cameraMatrix.at<double>(0, 1) = -4.1131;
        cameraMatrix.at<double>(0, 2) = 3.986573e+02;
        cameraMatrix.at<double>(1, 1) = 1.1914e+03;
        cameraMatrix.at<double>(1, 2) = 4.878128e+02;

        Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
        distCoeffs.at<double>(0, 0) = -0.8532;
        distCoeffs.at<double>(1, 0) = 0.8530;
        distCoeffs.at<double>(2, 0) = -0.0774;
        distCoeffs.at<double>(3, 0) = -0.0036;
        distCoeffs.at<double>(4, 0) = 0;

        cap >> frame;
        imageSize = frame.size();
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize, CV_16SC2, map1, map2);
    }

    while (nh.ok() && cap.isOpened())
    {
        cap >> frame;
        if (is_calibrate)
        {
            remap(frame, frameCalibration, map1, map2, INTER_LINEAR);
            imshow("Origianl", frame);
            imshow("Calibration", frameCalibration);
            char key = waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
                break;
        }
        if (frame.empty())
        {
            ROS_INFO("frame is empty!");
            return -1;
        }
        else
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}