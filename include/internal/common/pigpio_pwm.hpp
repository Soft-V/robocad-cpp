#pragma once

#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <vector>

// One controlling application at a time, sharing pigpiod's text FIFOs.
class PigpioPWM {
public:
    PigpioPWM(std::string pipe_path = "/dev/pigpio",
              std::string response_path = "/dev/pigout",
              int frequency = 100, double timeout = 1.0);
    void set_dutycycle(int gpio, double duty);
    // Removes pulses; releasing torque depends on the servo electronics.
    void stop(const std::vector<int>& gpios = {});

private:
    static constexpr int PWM_RANGE = 10000;
    std::string pipe_path, response_path;
    int frequency;
    double timeout;
    std::uint64_t pipe_device = 0, pipe_inode = 0;
    bool have_identity = false, stopped = false;
    std::set<int> configured, used;
    std::recursive_mutex mutex;
    int command(int command_fd, int response_fd, const std::string& text);
};
