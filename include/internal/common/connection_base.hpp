#pragma once

#include <opencv2/opencv.hpp>

class ConnectionBase 
{
public:
    virtual void stop();
    virtual cv::Mat get_camera();
    virtual std::vector<float> get_lidar();
};