#pragma once

#include "common/robot.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

class Shufflecad
{
public:
    static inline const std::string LOG_INFO = "info";
    static inline const std::string LOG_WARNING = "warning";
    static inline const std::string LOG_ERROR = "error";

    std::map<std::string, float> joystick_values;
    Robot* robot;

    Shufflecad(Robot* robot);
    ~Shufflecad();

    ShuffleVariable* add_var(ShuffleVariable* var);
    CameraVariable* add_var(CameraVariable* var);

    void print_to_log(std::string message, 
                      std::string message_type = LOG_INFO, 
                      std::string color = "#cccccc");

    void stop();

private:
    ConnectionHelper* connection_helper;
    std::vector<ShuffleVariable*> variables_array;
    std::vector<CameraVariable*> camera_variables_array;
    
    std::vector<std::string> print_array;
    std::vector<std::string> get_print_array();
    void clear_print_array();
};

class ShuffleVariable
{
public:
    static inline const std::string FLOAT_TYPE = "float";
    static inline const std::string STRING_TYPE = "string";
    static inline const std::string BIG_STRING_TYPE = "bigstring";
    static inline const std::string BOOL_TYPE = "bool";
    static inline const std::string CHART_TYPE = "chart";
    static inline const std::string SLIDER_TYPE = "slider";
    static inline const std::string RADAR_TYPE = "radar";

    static inline const std::string IN_VAR = "in";
    static inline const std::string OUT_VAR = "out";

    ShuffleVariable(std::string name, std::string type, std::string direction)
    {
        this->name = name;
        this->type = type;
        this->direction = direction;
        this->value = "";
    }

    void set_bool(bool value)
    {
        this->value = value ? "1" : "0";
    }
    void set_float(float value)
    {
        this->value = std::to_string(value);
    }
    void set_string(std::string value)
    {
        this->value = value;
    }
    void set_radar(std::vector<float> values)
    {
        std::string val = "";
        for (size_t i = 0; i < values.size(); i++) {
            val += std::to_string(i);
            val += std::to_string(values[i]);
            if (i != values.size() - 1) val += "+";
        }
        this->value = val;
    }

    bool get_bool()
    {
        return this->value == "1";
    }
    float get_float()
    {
        return std::stof(this->value);
    }
    std::string get_string()
    {
        return this->value;
    }

private:
    std::string name;
    std::string type;
    std::string direction;
    std::string value;
};

class CameraVariable
{
public:
    CameraVariable(std::string name)
    {
        this->name = name;
    }

    void set_mat(cv::Mat value)
    {
        this->value = value;
        this->width = value.cols;
        this->height = value.rows;
    }

    std::vector<uint8_t> get_value()
    {
        std::vector<uchar> result;
        cv::imencode(".jpg", this->value, result);
        return result;
    }

private:
    std::string name;
    cv::Mat value;
    int width;
    int height;
};

class ConnectionHelper
{
public:
    ConnectionHelper(Shufflecad* shufflecad, Robot* robot);
    ~ConnectionHelper();

    void stop();

private:
    Shufflecad* shufflecad;
    Robot* robot;

    void start();
};