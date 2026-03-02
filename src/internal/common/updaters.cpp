#include "updaters.hpp"

#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cstdio>

Updater::Updater(Robot* r)
{
	robot = r;
}

RpiUpdater::RpiUpdater(Robot* r) : Updater(r)
{
}

RepkaUpdater::RepkaUpdater(Robot* r) : Updater(r)
{
}

static float read_cpu_temperature() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) return 0.0f;
    std::string line;
    std::getline(file, line);
    file.close();
    try 
    {
        int millideg = std::stoi(line);
        return millideg / 1000.0f;
    } 
    catch (...) 
    {
        return 0.0f;
    }
}

static std::pair<long long, long long> read_cpu_stats() {
    std::ifstream stat("/proc/stat");
    if (!stat.is_open()) return {0, 0};
    std::string line;
    std::getline(stat, line);
    stat.close();
    long long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
    int count = std::sscanf(line.c_str(),
        "cpu %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld",
        &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal, &guest, &guest_nice);
    if (count < 4) return {0, 0};
    long long total = user + nice + system + idle + iowait + irq + softirq + steal;
    return {total, idle};
}

static float get_cpu_load() {
    auto stats1 = read_cpu_stats();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto stats2 = read_cpu_stats();
    float cpu_load = 0.0f;
    if (stats1.first != 0 && stats2.first != 0) 
    {
        long long total_diff = stats2.first - stats1.first;
        long long idle_diff = stats2.second - stats1.second;
        if (total_diff > 0) 
        {
            cpu_load = (total_diff - idle_diff) * 100.0f / total_diff;
        }
    }
    return cpu_load;
}

static float get_memory_load() {
    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo.is_open()) return 0.0f;
    long long total = 0, available = 0;
    std::string line;
    while (std::getline(meminfo, line)) 
    {
        if (line.rfind("MemTotal:", 0) == 0) 
        {
            std::sscanf(line.c_str(), "MemTotal: %lld kB", &total);
        } 
        else if (line.rfind("MemAvailable:", 0) == 0) 
        {
            std::sscanf(line.c_str(), "MemAvailable: %lld kB", &available);
        }
    }
    meminfo.close();
    if (total == 0) return 0.0f;
    long long used = total - available;
    return (used * 100.0f) / total;
}

void RpiUpdater::updater()
{
    while (!stop_robot_info_thread) 
    {
        try 
        {
            float cpu_load = get_cpu_load();
            float temperature = read_cpu_temperature();
            float memory_load = get_memory_load();

            {
                robot->robot_info->temperature = temperature;
                robot->robot_info->cpu_load = cpu_load;
                robot->robot_info->memory_load = memory_load;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        catch (const std::exception& e) 
        {
            robot->write_log(std::string("Info thread error: ") + e.what());
        }
        catch (...) 
        {
            robot->write_log("Info thread unknown error");
        }
    }
}

void RepkaUpdater::updater()
{
    while (!stop_robot_info_thread) 
    {
        try 
        {
            float cpu_load = get_cpu_load();
            float temperature = read_cpu_temperature();
            float memory_load = get_memory_load();

            {
                robot->robot_info->temperature = temperature;
                robot->robot_info->cpu_load = cpu_load;
                robot->robot_info->memory_load = memory_load;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        catch (const std::exception& e) 
        {
            robot->write_log(std::string("Info thread error: ") + e.what());
        }
        catch (...) 
        {
            robot->write_log("Info thread unknown error");
        }
    }
}