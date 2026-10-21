#include "internal/common/pigpio_pwm.hpp"
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    PigpioPWM pwm(argv[1], argv[2], 100, 0.15);
    const std::vector<int> gpios{4, 18, 17, 27, 23, 22, 24, 25, 7, 5};
    for (std::string line; std::getline(std::cin, line);) {
        std::istringstream input(line);
        std::string operation;
        input >> operation;
        try {
            if (operation == "set") {
                int gpio;
                std::string duty;
                input >> gpio >> duty;
                pwm.set_dutycycle(gpio, std::stod(duty));
            } else if (operation == "stop") pwm.stop(gpios);
            else if (operation == "stop-used") pwm.stop();
            else return 2;
            std::cout << "ok" << std::endl;
        } catch (const std::exception& error) {
            std::string message = error.what();
            for (auto& c : message) if (c == '\n' || c == '\r') c = ' ';
            std::cout << "error: " << message << std::endl;
        }
    }
}
