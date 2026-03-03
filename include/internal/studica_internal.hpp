#pragma once

#include "common/connection_base.hpp"
#include "common/robot.hpp"
#include "common/updaters.hpp"
#include "common/shared.hpp"
#include "common/robot_configuration.hpp"

class RobocadConnectionStudica;
class TitanCOMStudica;
class VMXSPIStudica;

class StudicaInternal 
{
public:
    const int HCDIO_CONST_ARRAY[10] = {4, 18, 17, 27, 23, 22, 24, 25, 7, 5};

    std::atomic<float> speed_motor_0 = 0.0f, speed_motor_1 = 0.0f, speed_motor_2 = 0.0f, speed_motor_3 = 0.0f;
    std::atomic<int32_t> enc_motor_0 = 0, enc_motor_1 = 0, enc_motor_2 = 0, enc_motor_3 = 0;
    std::atomic<uint16_t> raw_enc_motor_0 = 0, raw_enc_motor_1 = 0, raw_enc_motor_2 = 0, raw_enc_motor_3 = 0;

    std::atomic<float> yaw = 0.0f, yaw_unlim = 0.0f;
    std::atomic<float> ultrasound_1 = 0.0f, ultrasound_2 = 0.0f;
    std::atomic<int32_t> analog_1 = 0, analog_2 = 0, analog_3 = 0, analog_4 = 0;
    std::atomic<float> hcdio_values[10] = {0.0f};

    std::atomic<bool> limit_l_0 = false, limit_h_0 = false, limit_l_1 = false, limit_h_1 = false, limit_l_2 = false, limit_h_2 = false, limit_l_3 = false, limit_h_3 = false;
    std::atomic<bool> flex_0 = false, flex_1 = false, flex_2 = false, flex_3 = false, flex_4 = false, flex_5 = false, flex_6 = false, flex_7 = false;

    StudicaInternal(Robot* robot, RobotConfiguration* conf);
    ~StudicaInternal();

    void stop();
    cv::Mat get_camera();
    std::vector<float> get_lidar();

    void set_servo_angle(float angle, int pin);
    void set_led_state(bool state, int pin);
    void set_servo_pwm(float pwm, int pin);
    void disable_servo(int pin);

private:
    Robot* robot;
    ConnectionBase* connection;
    Updater* updater;

    RobocadConnectionStudica* robocad_connection;
    TitanCOMStudica* titan_com;
    VMXSPIStudica* vmx_spi;

    void echo_to_file(std::string s);
};