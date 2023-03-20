#include "camera_core.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "camera_node"); // 初始化节点，第三个参数 node_name
    std::cout << "camera_node init" << std::endl;
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 10);

    Mat image;
    VideoCapture capture;
    capture.open(0);
    ros::Rate loop_rate(15);
    if (capture.isOpened())
    {
        cout << "Capture is opened" << endl;
        int index = 0;
        while (nh.ok())
        {
            std_msgs::Header header;
            header.frame_id = "world";
            capture >> image;
            if (!image.empty())
            {
                stringstream str;
                // if (index % 10 == 0)
                // {
                //     str << "/home/kiana/Documents/images/" << index++ << ".png"; /*图片存储位置*/
                //     cout << str.str() << endl;
                //     imwrite(str.str(), image);
                // }

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
                pub.publish(msg);
                // cv::waitKey(1);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else
    {
        cout << "No capture" << endl;
    }

    return 0;
}