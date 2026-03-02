#pragma once

#include <opencv2/opencv.hpp>

class ConnectionBase 
{
public:
    virtual void stop();
    virtual cv::Mat get_camera();
    virtual float* get_lidar();
};