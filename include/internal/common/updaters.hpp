#pragma once

#include "robot.hpp"

class Updater 
{
public:
    Robot* robot;
    std::atomic<bool> stop_robot_info_thread = false;

    Updater(Robot* robot);

    virtual void updater();
};

class RpiUpdater : public Updater
{
public:
    RpiUpdater(Robot* robot);

    void updater() override;
};

class RepkaUpdater : public Updater
{
public:
    RepkaUpdater(Robot* robot);

    void updater() override;
};