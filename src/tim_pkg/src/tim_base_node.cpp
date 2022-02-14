//
// Created by Kiana on 22-2-10.
//

#include "tim_base_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tim_base");  //初始化节点，第三个参数 node_name

    ros::NodeHandle nh;

    TimBaseCore core(nh);
    // core.Spin();
    return 0;
}