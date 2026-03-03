#include "shufflecad.hpp"

#include <csignal>
#include <stdio.h>

Shufflecad* current_instance = nullptr;
void handler(int signum)
{
    current_instance->robot->write_log("Program stopped from handler");
    current_instance->robot->write_log("Signal handler called with signal " + std::to_string(signum));
    current_instance->stop();
    exit(signum);
}

Shufflecad::Shufflecad(Robot* robot) : robot(robot) 
{
    // Set the global instance pointer for signal handling
    current_instance = this;
    
    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    connection_helper = new ConnectionHelper(this, robot);
}

Shufflecad::~Shufflecad()
{
    // delete connection helper
    delete connection_helper;
    connection_helper = NULL;
}

void Shufflecad::stop() 
{
    // stop connection helper
    connection_helper->stop();
}

ShuffleVariable* Shufflecad::add_var(ShuffleVariable* var)
{
    variables_array.push_back(var);
    return var;
}

CameraVariable* Shufflecad::add_var(CameraVariable* var)
{
    camera_variables_array.push_back(var);
    return var;
}

void Shufflecad::print_to_log(std::string message, std::string message_type, std::string color)
{
    std::string formatted_message = message_type + "@" + message + color;
    print_array.push_back(formatted_message);
}

std::vector<std::string> Shufflecad::get_print_array()
{
    return print_array;
}
void Shufflecad::clear_print_array()
{
    print_array.clear();
}