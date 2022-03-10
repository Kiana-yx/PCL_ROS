#include "grid_core.h"

GridCore::GridCore(ros::NodeHandle &nh)
{
    SetMapParams();
    sub_point_cloud_ = nh.subscribe("/filtered_points_no_ground", 10, &GridCore::grid_main, this);

    pub_2d_ = nh.advertise<sensor_msgs::PointCloud2>("/g2d_points", 10);
    pub_grid_ = nh.advertise<nav_msgs::OccupancyGrid>("/grid_points", 10);

    //程序到达ros::spin()之前按照一系列规则，设定一系列话题订阅者
    // ros::spin()使订阅者们可以开始接受话题，进入回调函数
    ros::spin();
}

GridCore::~GridCore() {}

void GridCore::Spin()
{
}

void GridCore::publish_cloud(const ros::Publisher &in_publisher,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                             const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void GridCore::Point_2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(1.0);
    coefficients->values.push_back(0.0);

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setInputCloud(in);
    proj.setModelCoefficients(coefficients);
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.filter(*out);
}

//设置地图参数
void GridCore::SetMapParams(void)
{
    //地图大小、分辨率
    mapParams.width = width_x;
    mapParams.height = width_y;       //单位栅格个数
    mapParams.resolution = grid_size; // 0.04m 1/0.04=25个栅格
    //假设free=-1、occ=2
    mapParams.log_free = -1;
    mapParams.log_occ = 2;

    //栅格坐标原点相对于世界坐标原点
    mapParams.origin_x = map_x0; // 0
    mapParams.origin_y = map_y0; //-10

    // TODO:
    mapParams.offset_x = 0;
    mapParams.offset_y = 0; //单位栅格个数
    //为地图指针申请空间
    pMap = new unsigned char[mapParams.width * mapParams.height];

    //每一个栅格代表的值，初始化为50
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
        pMap[i] = 50;
}

//目的：将机器人的实际位置，在900x900的栅格地图中找到对应的栅格序号，返回GridIndex对象
GridIndex GridCore::ConvertWorld2GridIndex(double x, double y)
{
    GridIndex index;
    // ceil()向上取整函数
    index.x = round((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = round((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

int GridCore::GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y * mapParams.width + index.x;
}

// 2D画线算法　来进行计算两个点之间的grid cell
//目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
std::vector<GridIndex> GridCore::TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            pointX = y;
            pointY = x;
        }
        else
        {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点．
        if (pointX == x1 && pointY == y1)
            continue;

        //保存所有的点
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }

    return gridIndexVector;
}

//占据栅格地图构建算法
//输入激光雷达数据和机器人位姿数据
//目的：通过遍历所有帧数据，为pMap[]中的每个穿过的空闲栅格或者击中栅格赋新值，中间有个计算方法，也就是占用栅格地图构建的理论实现
// void GridCore::OccupanyMapping(const pcl::PointCloud<pcl::PointXYZ>::Ptr in, std::vector<Eigen::Vector3d> &robot_poses)
void GridCore::OccupanyMapping(const pcl::PointCloud<pcl::PointXYZ>::Ptr in)
{
    Eigen::Vector3d robotPose = {0, 0, 0}; //目前认为雷达留守世界坐标系原点（0，0）

    //获取该帧机器人位姿的栅格序号
    GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

    //遍历该帧激光雷达数据所有扫描点
    for (int id = 0; id < in->points.size(); id++)
    {
        //得到该激光扫描点，在世界坐标系下的位置
        double world_x = in->points[id].x + robotPose(0);
        double world_y = in->points[id].y + robotPose(1);

        //将该激光扫描点在世界坐标系下的位置，转化为栅格序号
        GridIndex mapIndex = ConvertWorld2GridIndex(world_x, world_y);

        //从机器人的栅格序号到该激光扫描点的栅格序号划线
        //目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
        std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x, robotIndex.y, mapIndex.x, mapIndex.y);

        //遍历该扫描激光点通过的所有空闲栅格
        for (int k = 0; k < freeIndex.size(); k++)
        {
            GridIndex tmpIndex = freeIndex[k];
            //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
            int linearIndex = GridIndexToLinearIndex(tmpIndex);
            //取出该栅格代表的数据
            int data = pMap[linearIndex];
            //根据栅格空闲规则，执行data += mapParams.log_free;
            if (data > 0)                   //默认data=50
                data += mapParams.log_free; // log_free=-1，data将变小
            else
                data = 0;
            //给该空闲栅格赋新值，最小为0
            pMap[linearIndex] = data;
        }

        //更新该激光扫描点集中的栅格，
        int tmpIndex = GridIndexToLinearIndex(mapIndex);
        int data = pMap[tmpIndex];
        //根据栅格击中规则，执行data += mapParams.log_occ;
        if (data < 100)                //默认data=50
            data += mapParams.log_occ; // log_occ=2，data将变大
        else
            data = 100;
        //给击中的栅格赋新值，最大100
        pMap[tmpIndex] = data;
        //到这里，对一个位姿下的一个激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
    }
    //到这里，对一个位姿下的一帧激光扫描数据经过的空闲栅格和击中栅格进行了重新赋值
}

//发布地图．
void GridCore::PublishMap(ros::Publisher &map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    // 0~100
    for (int i = 0; i < mapParams.width * mapParams.height; i++)
    {
        if (pMap[i] == 50) //未知栅格
        {
            rosMap.data[i] = -1.0;
        }
        else if (pMap[i] < 50) //空闲栅格
        {
            //  rosMap.data[i] = 0;   //gmapping    方式
            rosMap.data[i] = pMap[i]; // cartographer方式
        }
        else if (pMap[i] > 50) //击中栅格
        {
            //  rosMap.data[i] = 100;
            rosMap.data[i] = pMap[i];
        }
    }
    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "velodyne";

    map_pub.publish(rosMap);
}

void GridCore::grid_create(const pcl::PointCloud<pcl::PointXYZ>::Ptr in)
{
    // 定义地图对象
    nav_msgs::OccupancyGrid myMap;

    // 地图数据
    myMap.info.map_load_time = ros::Time(0);
    myMap.info.resolution = grid_size;
    myMap.info.width = width_x;
    myMap.info.height = width_y;
    myMap.info.origin.position.x = map_x0;
    myMap.info.origin.position.y = map_y0;
    myMap.info.origin.position.z = map_z0;
    myMap.info.origin.orientation.x = 0;
    myMap.info.origin.orientation.y = 0;
    myMap.info.origin.orientation.z = 0;
    myMap.info.origin.orientation.w = 1;

    int p[myMap.info.width * myMap.info.height] = {-1}; // [0,100]

    for (int count = 0; count < in->points.size(); count++)
    {
        int x_grid = round((in->points[count].x - map_x0) / grid_size) + 1;
        int y_grid = round((in->points[count].y - map_y0) / grid_size) + 1;

        int grid = (y_grid - 1) * width_x + x_grid - 1;
        p[grid] = 100;
    }

    std::vector<signed char> a(p, p + width_x * width_y);
    myMap.data = a;

    // frame id
    myMap.header.frame_id = "velodyne";
    // 广播
    pub_grid_.publish(myMap);
}

void GridCore::grid_main(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_3D_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_2D_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud_header_ = in_cloud_ptr->header;
    pcl::fromROSMsg(*in_cloud_ptr, *current_3D_ptr);

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(current_3D_ptr);
    filter.setLeafSize(grid_size, grid_size, 2 * grid_size);
    filter.filter(*current_3D_ptr);

    Point_2D(current_3D_ptr, point_2D_ptr);
    // publish_cloud(pub_2d_, point_2D_ptr, in_cloud_ptr->header);
    // grid_create(point_2D_ptr);

    //占用栅格地图构建算法
    OccupanyMapping(point_2D_ptr);
    //发布map，可视化
    PublishMap(pub_grid_);
    
    if(pMap != NULL)
    delete pMap;
}
