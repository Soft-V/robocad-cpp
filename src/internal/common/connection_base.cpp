#include "connection_base.hpp"

#include <opencv2/opencv.hpp>

void ConnectionBase::stop()
{
}

cv::Mat ConnectionBase::get_camera()
{
    cv::Mat frame;
    return frame;
}

std::vector<float> ConnectionBase::get_lidar()
{
    return std::vector<float>();
}