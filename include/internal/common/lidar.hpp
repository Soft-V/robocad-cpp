#pragma once

#include <string>
#include <stdint.h>
#include <vector>
#include "robot.hpp"

class YDLidarX2 
{
public:
    YDLidarX2(Robot* robot, std::string port);
    ~YDLidarX2();

    void connect();
    void start_scan();
    void stop_scan();
    void disconnect();

    std::vector<float> get_data();
private:
    Robot* robot;
    std::string port;
};