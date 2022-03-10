#include "readfile.h"

#include <fstream>
#include <boost/algorithm/string.hpp>

//字符串转数字，适应于不同类型的数字，int、double
template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;

    return num;
}

//别名
typedef std::string::size_type string_size;

//根据分隔符，分离字符串，返回std::vector<std::string>
//目的：将字符串按照分隔符分开，放到数组中
std::vector<std::string> splitString(const std::string &s, const std::string &seperator)
{
    std::vector<std::string> result;
    string_size i = 0;
    //遍历所有字符串
    while(i != s.size())
    {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while(i != s.size() && flag == 0)
        {
            flag = 1;
            for(string_size k = 0; k < seperator.size(); k++)
            {
                if(s[i] == seperator[k])
                {
                    i++;
                    flag = 0;
                    break;
                }
            }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while(j != s.size() && flag == 0)
        {
            for(string_size k = 0; k < seperator.size(); k++)
            {
                if(s[j] == seperator[k])
                {
                    flag = 1;
                    break;
                }
            }
            if(flag == 0)
                j++;
        }

        if(i != j)
        {
            result.push_back(s.substr(i, j-i));
            i = j;
        }
    }
    return result;
}

//读取机器人的位姿信息．
//目的：从指定路径下的文件中，读取位姿数据，存入Eigen::Vector3d类型的数组中
void ReadPoseInformation(const std::string path,std::vector<Eigen::Vector3d>& poses)
{
    std::ifstream fin(path.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read File Failed!!!"<<std::endl;
        return ;
    }
    //位姿个数计数
    int cnt = 0;
    //存放每一行字符串的对象
    std::string line;
    //遍历所有行
    while(std::getline(fin,line))
    {
        cnt++;
        std::vector<std::string> results;
        //每一行只分离出3个字符串
        results = splitString(line,",");
        //三个字符串分别对应，x,y,theta
        double x = stringToNum<double>(results[0]);
        double y = stringToNum<double>(results[1]);
        double theta = stringToNum<double>(results[2]);

        Eigen::Vector3d pose(x,y,theta);
        //存入Eigen::Vector3d类型的数组中
        poses.push_back(pose);
        //使用位姿的个数限制，自定义
        if(cnt >= READ_DATA_NUMBER)
            break;

    }
    fin.close();

    std::cout <<"Read Pose Good!!!"<<std::endl;
}

//读取机器人的激光雷达信息
//目的：分别从两个路径下的文件中读出数据，存入到GeneralLaserScan的数组中，GeneralLaserScan的一个对象，包含每一帧所有激光点的距离和对应的角度
void ReadLaserScanInformation(const std::string anglePath,
                              const std::string laserPath,
                              std::vector< GeneralLaserScan >& laserscans)
{
    //////////////////////////////////////////读取角度信息///////////////////////////////////////////////
    std::ifstream fin(anglePath.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read Angle File Failed!!!"<<std::endl;
        return ;
    }

    GeneralLaserScan tmpGeneralLaserScan;

    //读取角度信息
    int cnt = 0;
    std::string line;
    std::getline(fin,line);
    std::vector<std::string> results;
    //这里angle数据只有一行，所以没有遍历所有行的循环；同时也说明这里假设每一帧相对应激光点的角度都一致，实际上不一定一致。
    results = splitString(line,",");
    for(int i = 0; i < results.size();i++)
    {
        double anglei = stringToNum<double>(results[i]);
        //将该行所有字符串，存入tmpGeneralLaserScan.angle_readings中
        tmpGeneralLaserScan.angle_readings.push_back(anglei);
    }
    fin.close();

    std::cout <<"Read Angle good:"<<tmpGeneralLaserScan.angle_readings.size()<<std::endl;

    //////////////////////////////////////////读取激光信息///////////////////////////////////////////////

    fin.open(laserPath.c_str());
    if(fin.is_open() == false)
    {
        std::cout <<"Read Scan File Failed!!!"<<std::endl;
        return ;
    }
    //将上一帧的laserscans数据清空
    laserscans.clear();

    //遍历所有行字符串，每一行距离数据个数和上面角度数据个数一致
    while(std::getline(fin,line))
    {
        //读取一行，每一行进行分割
        std::vector<std::string> results;
        results = splitString(line,",");

        cnt++;
        //清空tmpGeneralLaserScan.range_readings数据，可以认为是初始化
        tmpGeneralLaserScan.range_readings.clear();
        //将每一行（每一帧）的激光雷达数据，存入tmpGeneralLaserScan.range_readings中
        for(int i = 0; i < results.size();i++)
        {
            double rangei = stringToNum<double>(results[i]);
            tmpGeneralLaserScan.range_readings.push_back(rangei);
        }
        //将每一帧激光雷达数据的，角度、距离，存入laserscans中
        laserscans.push_back(tmpGeneralLaserScan);
        //使用激光的帧数限制，自定义
        if(cnt >= READ_DATA_NUMBER)
            break;
    }

    fin.close();

    std::cout <<"Read Laser Scans Good!!!!"<<std::endl;
}







