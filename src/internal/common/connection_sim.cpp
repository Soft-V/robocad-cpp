#include "connection_sim.hpp"

#include <opencv2/opencv.hpp>
#include <cstdlib>

ConnectionSim::ConnectionSim(Robot* robot)
    : robot(robot)
{
    talk_channel = new TalkPort(robot, port_set_data);
    talk_channel->start_talking();
    listen_channel = new ListenPort(robot, port_get_data);
    listen_channel->start_listening();
    camera_channel = new ListenPort(robot, port_camera);
    camera_channel->start_listening();
}

ConnectionSim::~ConnectionSim()
{
    delete talk_channel;
    talk_channel = NULL;
    delete listen_channel;
    listen_channel = NULL;
    delete camera_channel;
    camera_channel = NULL;
}

void ConnectionSim::stop()
{
    if (talk_channel)
        talk_channel->stop_talking();
    if (listen_channel)
        listen_channel->stop_listening();
    if (camera_channel)
        camera_channel->stop_listening();
}

cv::Mat ConnectionSim::get_camera()
{
    if (!camera_channel)
        return cv::Mat();
    std::vector<uint8_t> data = camera_channel->out_bytes;
    if (data.size() >= 921600)
    {
        cv::Mat img(480, 640, CV_8UC3, data.data());
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        return img.clone();
    }
    return cv::Mat();
}

float* ConnectionSim::get_lidar()
{
    return NULL;
}