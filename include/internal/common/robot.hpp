#pragma once

#include "robot_configuration.hpp"
#include <string>
#include <mutex>
#include <fstream>

class RobotInfo
{
public:
	float spi_time_dev = 0;
	float rx_spi_time_dev = 0;
	float tx_spi_time_dev = 0;
	float spi_count_dev = 0;
	float com_time_dev = 0;
	float rx_com_time_dev = 0;
	float tx_com_time_dev = 0;
	float com_count_dev = 0;
	float temperature = 0;
	float memory_load = 0;
	float cpu_load = 0;
};

class Robot 
{
public:
	bool on_real_robot;
	float power;
	RobotInfo* robot_info;

	Robot(bool on_real_robot, RobotConfiguration* conf);
	virtual ~Robot();

	void write_log(std::string text);

protected:
	std::ofstream log_file;
	std::mutex log_mutex;
};