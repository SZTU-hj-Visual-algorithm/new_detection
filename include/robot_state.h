//
// Created by liyankuan on 2022/4/17.
//

#ifndef DEMO_ROBOT_STATE_H
#define DEMO_ROBOT_STATE_H

enum EnermyColor { RED = 0, BLUE = 1 };

class robot_state
{
public:
	float ab_pitch = 0.0;
	float ab_yaw = 0.0;
	float ab_roll = 0.0;
	float SPEED = 25.0;
	int enermy_color;
};

#endif //DEMO_ROBOT_STATE_H
