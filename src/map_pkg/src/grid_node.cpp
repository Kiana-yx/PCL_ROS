#include "grid_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid");  //初始化节点，第三个参数 node_name

    ros::NodeHandle nh;

    GridCore core(nh);

    return 0;
}