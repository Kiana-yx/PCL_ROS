#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

/***********************************************************************************************************
  占据栅格地图构建算法
  由公众号：小白学移动机器人，对该代码进行详细中文注释，以及对核心代码进行更改
  欢迎关注公众号，从此学习的路上变得不再孤单，加油！奥利给！！！
***********************************************************************************************************/

//2D画线算法　来进行计算两个点之间的grid cell
//目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
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
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}
//设置地图参数
void SetMapParams(void )
{
   //地图大小、分辨率
   mapParams.width = 900;
   mapParams.height = 900;     //单位栅格个数
   mapParams.resolution = 0.04;//0.04m 1/0.04=25个栅格
   //假设free=-1、occ=2
   mapParams.log_free = -1;
   mapParams.log_occ = 2;
   //
   mapParams.origin_x = 0.0;
   mapParams.origin_y = 0.0;

   //地图的原点，即是机器人默认位置
   mapParams.offset_x = 700;
   mapParams.offset_y = 600;  //单位栅格个数
   //为地图指针申请空间
   pMap = new unsigned char[mapParams.width*mapParams.height];

   //每一个栅格代表的值，初始化为50
   for(int i = 0; i < mapParams.width * mapParams.height;i++)
        pMap[i] = 50;
}

//从世界坐标系转换到栅格坐标系，主要是存在一个分辨率
//比如resolution = 0.04，世界坐标系下，单位1在栅格坐标系可以表示1/resolution=25个栅格
//目的：将机器人的实际位置，在900x900的栅格地图中找到对应的栅格序号，返回GridIndex对象
GridIndex ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;
    //ceil()向上取整函数
    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

//从栅格序号，转化到数组序号；因为栅格最终是按照顺序（width从小到大，height从低到高）依次存储到动态数组中的
int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
}

//判断index是否有效
//目的：判断该栅格序号是否在设定栅格地图大小范围内
bool isValidGridIndex(GridIndex index)
{
    if(index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

//销毁地图指针
void DestoryMap()
{
    if(pMap != NULL)
        delete pMap;
}

//占据栅格地图构建算法
//输入激光雷达数据和机器人位姿数据
//目的：通过遍历所有帧数据，为pMap[]中的每个穿过的空闲栅格或者击中栅格赋新值，中间有个计算方法，也就是占用栅格地图构建的理论实现
void OccupanyMapping(std::vector<GeneralLaserScan>& scans,std::vector<Eigen::Vector3d>& robot_poses)
{
  std::cout <<"Scans Size:"<<scans.size()<<std::endl;
  std::cout <<"Poses Size:"<<robot_poses.size()<<std::endl;

  //遍历所有帧激光雷达数据
  for(int i = 0; i < scans.size();i++)
  {
    //获取每一帧的激光雷达、机器人位姿数据
    GeneralLaserScan scan = scans[i];
    Eigen::Vector3d robotPose = robot_poses[i];

    //获取该帧机器人位姿的栅格序号
    GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0),robotPose(1));

    //判断该帧机器人位姿的栅格序号，是否在自己设定的栅格地图范围内
    if(isValidGridIndex(robotIndex) == false) continue;
  
    //遍历该帧激光雷达数据所有扫描点
    for(int id = 0; id < scan.range_readings.size();id++)
    {
      //取出该激光雷达扫描点的距离和角度
      double dist = scan.range_readings[id];
      double angle = scan.angle_readings[id];
      //剔除异常数据，跳过该次循环，不作处理
      if(std::isinf(dist) || std::isnan(dist)) continue;
      //机器人航向角，机器人x轴与世界坐标系x轴夹角
      double theta = robotPose(2);

      //在旋转过后（与世界坐标系（像素坐标系下）平行）的激光雷达坐标系下的坐标x,y
      //该开始一直不理解这个为啥laser_y要加一个负号
      //明确激光雷达数据的角度逆时针变化
      //明确机器人航向角与世界坐标系x轴呈逆时针变化
      //明确这里的世界坐标系world_x，不是真实的世界坐标系，而是像素坐标系，y轴与真实的世界坐标系相反，这样是laser_y加负号的原因
      double laser_x =   dist * cos(theta + angle);
      double laser_y =  -dist * sin(theta + angle);

      //得到该激光扫描点，在世界坐标系下（像素坐标系下）的位置
      double world_x = laser_x + robotPose(0);
      double world_y = laser_y + robotPose(1);

      //将该激光扫描点在世界坐标系下的位置，转化为栅格序号
      GridIndex mapIndex = ConvertWorld2GridIndex(world_x,world_y);

      //判断该激光扫描点的栅格序号，是否在自己设定的栅格地图900x900范围内，如果不在则跳过
      if(isValidGridIndex(mapIndex) == false)continue;

      //从机器人的栅格序号到该激光扫描点的栅格序号划线
      //目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
      std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x,robotIndex.y,mapIndex.x,mapIndex.y);

      //遍历该扫描激光点通过的所有空闲栅格
      for(int k = 0; k < freeIndex.size();k++)
      {
        GridIndex tmpIndex = freeIndex[k];
        //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
        int linearIndex = GridIndexToLinearIndex(tmpIndex);
        //取出该栅格代表的数据
        int data = pMap[linearIndex];
        //根据栅格空闲规则，执行data += mapParams.log_free;
        if(data > 0)//默认data=50
          data += mapParams.log_free;//log_free=-1，data将变小
        else
          data = 0;
        //给该空闲栅格赋新值，最小为0
        pMap[linearIndex] = data;

      }

      //更新该激光扫描点集中的栅格，
      int tmpIndex = GridIndexToLinearIndex(mapIndex);
      int data = pMap[tmpIndex];
      //根据栅格击中规则，执行data += mapParams.log_occ;
      if(data < 100)//默认data=50
        data += mapParams.log_occ;//log_occ=2，data将变大
      else
        data = 100;
      //给击中的栅格赋新值，最大100
      pMap[tmpIndex] = data;
      //到这里，对一个位姿下的一个激光扫描数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
    }
    //到这里，对一个位姿下的一帧激光扫描数据经过的空闲栅格和击中栅格进行了重新赋值
  }
  //到这里，对所有帧激光扫描数据经过的空闲栅格和击中栅格进行了重新赋值
}

//发布地图．
void PublishMap(ros::Publisher& map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    //0~100
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
    {
       if(pMap[i] == 50)    //未知栅格
       {
           rosMap.data[i] = -1.0;
       }
       else if(pMap[i] < 50)//空闲栅格
       {
          //  rosMap.data[i] = 0;   //gmapping    方式
           rosMap.data[i] = pMap[i];//cartographer方式
       }
       else if(pMap[i] > 50)//击中栅格
       {
          //  rosMap.data[i] = 100;
           rosMap.data[i] = pMap[i];
       }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OccupanyMapping");

  ros::NodeHandle nodeHandler;

  ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);

  std::vector<Eigen::Vector3d> robotPoses;
  std::vector<GeneralLaserScan> generalLaserScans;

  std::string basePath = "/home/kiana/catkin_ws/src/occupany_mapping/data";

  std::string posePath= basePath + "/pose.txt";
  std::string anglePath = basePath + "/scanAngles.txt";
  std::string scanPath = basePath + "/ranges.txt";

  //读取机器人位姿数据
  ReadPoseInformation(posePath,robotPoses);
  //读取激光雷达数据
  ReadLaserScanInformation(anglePath,
                            scanPath,
                            generalLaserScans);

  //设置地图信息
  SetMapParams();
  //占用栅格地图构建算法
  OccupanyMapping(generalLaserScans,robotPoses);
  //发布map，可视化
  PublishMap(mapPub);
  //ROS消息回调处理函数
  //ros::spin();调用后不会再执行之后的程序，一般在非循环程序中;
  //ros::spinOnce();调用后还可以继续执行之后的程序，一般用在循环程序中
  ros::spin();
  //销毁地图
  DestoryMap();

  std::cout <<"Release Memory!!"<<std::endl;

}




