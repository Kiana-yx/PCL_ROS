#include "grid_core.h"

GridCore::GridCore(ros::NodeHandle &nh)
{
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
        p[grid]=100;
    }

    std::vector<signed char> a(p, p + width_x*width_y);
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
    publish_cloud(pub_2d_, point_2D_ptr, in_cloud_ptr->header);
    grid_create(point_2D_ptr);
}
