#include "lidar.hpp"

#include <vector>

// TODO:

YDLidarX2::YDLidarX2(Robot* robot, std::string port) : robot(robot), port(port) 
{
}

YDLidarX2::~YDLidarX2()
{
}

void YDLidarX2::connect()
{
}

void YDLidarX2::start_scan()
{
}

void YDLidarX2::stop_scan()
{
}

void YDLidarX2::disconnect()
{
}

std::vector<float> YDLidarX2::get_data()
{
    return std::vector<float>();
}