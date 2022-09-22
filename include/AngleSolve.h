//
// Created by 蓬蒿浪人 on 2022/9/16.
//

#ifndef SHSIHI_ANGLESOLVE_H
#define SHSIHI_ANGLESOLVE_H

#include "robot_state.h"

struct headAngle
{
    double yaw;
    double pitch;
};

class AngleSolve
{
public:
    void getAngle();

private:
    headAngle send;
};

#endif //SHSIHI_ANGLESOLVE_H
