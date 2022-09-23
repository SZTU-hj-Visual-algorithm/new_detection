//
// Created by liyankuan on 2022/4/17.
//

#ifndef DEMO_ROBOT_STATE_H
#define DEMO_ROBOT_STATE_H

enum EnermyColor { RED = 0, BLUE = 1 };
enum EnermyType  { SMALL = 0, BIG = 1 };
enum SpinHeading {UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE};


class robot_state
{
public:
    //电控发来的角度和弹速
	float ab_pitch = 0.0;
	float ab_yaw = 0.0;
	float ab_roll = 0.0;
	float SPEED = 25.0;
	int enermy_color;
    int enermy_type;
    int enermy_ID;
};

#endif //DEMO_ROBOT_STATE_H
