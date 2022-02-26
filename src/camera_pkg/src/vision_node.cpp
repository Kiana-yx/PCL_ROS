#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image_source1, const ImageConstPtr& image_source2)
{
  setlocale(LC_CTYPE, "zh_CN.utf8");  
  ROS_INFO("成功接受!");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_sub(nh, "/camera/color/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  TimeSynchronizer<Image, Image> sync(image_sub, depth_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
