
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

class Frame
{
private:
    /* data */
public:
    float min_x;
    float min_y;
    float min_z;
    float max_x;
    float max_y;
    float max_z;
    geometry_msgs::Point p[8];
    void get_param(void);
    void get_point(void);
};

void Frame::get_param(void)
{
    ros::param::get("/min_x", min_x);
    ros::param::get("/min_y", min_y);
    ros::param::get("/min_z", min_z);
    ros::param::get("/max_x", max_x);
    ros::param::get("/max_y", max_y);
    ros::param::get("/max_z", max_z);
}

void Frame::get_point(void)
{
    p[0].x = min_x;
    p[0].y = min_y;
    p[0].z = min_z;
    p[1].x = max_x;
    p[1].y = min_y;
    p[1].z = min_z;
    p[2].x = max_x;
    p[2].y = max_y;
    p[2].z = min_z;
    p[3].x = min_x;
    p[3].y = max_y;
    p[3].z = min_z;

    p[4].x = min_x;
    p[4].y = min_y;
    p[4].z = max_z;
    p[5].x = max_x;
    p[5].y = min_y;
    p[5].z = max_z;
    p[6].x = max_x;
    p[6].y = max_y;
    p[6].z = max_z;
    p[7].x = min_x;
    p[7].y = max_y;
    p[7].z = max_z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("safety_frame", 10);

    ros::Rate r(10); //设置循环频率为10hz

    Frame frame1;

    while (ros::ok())
    {
        frame1.get_param();
        frame1.get_point();

        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "velodyne";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.scale.x = 0.05;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // Line list is red
        line_list.color.g = 1.0;
        line_list.color.a = 1.0;

        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < 4; ++i)
        {
            line_list.points.push_back(frame1.p[i % 4]);
            line_list.points.push_back(frame1.p[(i + 1) % 4]);
        }
        line_list.points.push_back(frame1.p[0]);
        line_list.points.push_back(frame1.p[4]);
        for (uint32_t i = 4; i < 7; ++i)
        {
            line_list.points.push_back(frame1.p[i]);
            line_list.points.push_back(frame1.p[i + 1]);
        }
        line_list.points.push_back(frame1.p[7]);
        line_list.points.push_back(frame1.p[4]);
        for (uint32_t i = 0; i < 4; ++i)
        {
            line_list.points.push_back(frame1.p[i]);
            line_list.points.push_back(frame1.p[i + 4]);
        }

        marker_pub.publish(line_list);

        r.sleep();
    }
}