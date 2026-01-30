#include <iostream>
#include <csignal>
#include "dg_control/control.hpp"
#include <chrono>

using namespace handcontrol;

DGControl* dg = DGControl::getInstance();
bool mainprog = true;

void signalHandler(int)
{
    mainprog = false;
}

int main()
{   
    std::signal(SIGINT, signalHandler);

    Eigen::Array<double,MAX_JOINT_COUNT,1> pos;
    Eigen::Array<double,MAX_JOINT_COUNT,1> cur;
    Eigen::Array<double,MAX_JOINT_COUNT,1> vel;
    Eigen::Array<double,MAX_JOINT_COUNT,1> temp;

    Eigen::Array<double,MAX_JOINT_COUNT,1> target_pos1;
    Eigen::Array<double,MAX_JOINT_COUNT,1> target_pos2;

    pos << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    cur << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    vel << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    temp << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

    target_pos1 << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    target_pos2 << 0,0,50,50,
                0,50,80,80,
                0,0,0,0,
                0,50,80,80,
                0,0,80,80;
    
    dg->start();

    int t = 0;

    while(mainprog)
    {   
        ++t;

        if(t < 20000)
        {
            dg->setTragetPosition(target_pos1);
        }
        else if(t < 40000)
        {
            dg->setTragetPosition(target_pos2);
        }
        else
        {
            t=0;
        }

        dg->getCurrentPosition(pos);

        dg->getCurrentCurrent(cur);
        dg->getCurrentVelocity(vel);
        dg->getCurrentTemperature(temp);

        // std::cout << t << "\n";

        std::this_thread::sleep_for(std::chrono::microseconds(100));

    }

    dg->stop();

    return 0;
}