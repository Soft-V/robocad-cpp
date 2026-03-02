#pragma once

#include "connection_base.hpp"
#include "robot.hpp"
#include "updaters.hpp"
#include "shared.hpp"
#include "robot_configuration.hpp"
#include "connection.hpp"

#include <thread>
#include <cstdlib>

class ConnectionSim : public ConnectionBase
{
public:
    ConnectionSim(Robot* robot);
    ~ConnectionSim();
    void stop() override;
    cv::Mat get_camera() override;
    float* get_lidar() override;

    uint8_t* get_data();
    void set_data(uint8_t* data, unsigned int len);

private:
    const int port_set_data = 65431;
    const int port_get_data = 65432;
    const int port_camera = 65438;

    TalkPort* talk_channel;
    ListenPort* listen_channel;
    ListenPort* camera_channel;

    Robot* robot;
    std::thread* robot_info_thread;
};