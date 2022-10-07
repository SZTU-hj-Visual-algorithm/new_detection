//
// Created by liyankuan on 2022/4/17.
//

#ifndef DEMO_ROBOT_STATE_H
#define DEMO_ROBOT_STATE_H

//basic parameter
#define GRAVITY 9.78
#define SMALL_AIR_K 0.01903
#define BIG_AIR_K 0.00556
#define BIG_LIGHT_AIR_K 0.00530

//robot basic classes
enum EnermyColor { RED = 1, BLUE = 2 };
enum EnermyType  { SMALL = 1, BIG = 2 };
enum SpinHeading {UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE};

//robot state information from electronic control group
class robot_state
{
public:
    //电控发来的角度和弹速
    float ab_pitch = 0.0;
    float ab_yaw = 0.0;
    float ab_roll = 0.0;
    float SPEED = 25.0;
    int enermy_color = BLUE;
    int enermy_type;
    int enermy_ID;
};

#endif //DEMO_ROBOT_STATE_H