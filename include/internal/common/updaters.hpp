#pragma once

#include "robot.hpp"

class Updater 
{
public:
    Robot* robot;
    bool stop_robot_info_thread = false;

    Updater(Robot* robot);

    virtual void updater();
};

class RpiUpdater : Updater
{
public:
    RpiUpdater(Robot* robot);

    void updater() override;
};

class RepkaUpdater : Updater
{
public:
    RepkaUpdater(Robot* robot);

    void updater() override;
};