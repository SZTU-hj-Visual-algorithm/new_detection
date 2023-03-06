#include "robot_status.h"
//namespace robot_detection{

//void robot_state::updateData(float *data,int color)
//{
//    ab_pitch = data[0];
//    ab_yaw = data[1];
////    ab_roll = data[2];
//    bullet_speed = data[3];
//    enemy_color = color;
//}

void robot_state::updateData(const float *data, const float *quat)
{
    ab_pitch = data[0];
    ab_yaw = data[1];
//    ab_roll = data[2];
    bullet_speed = data[3];
	bullet_speed = 28;
	
	quaternion[0] = quat[0];
	quaternion[1] = quat[1];
	quaternion[2] = quat[2];
	quaternion[3] = quat[3];
}

void robot_state::clone(robot_state &robot)
{
    ab_pitch = robot.ab_pitch;
    ab_yaw = robot.ab_yaw;
//    ab_roll = robot.ab_roll;
    bullet_speed = robot.bullet_speed;
    quaternion[0] = robot.quaternion[0];
	quaternion[1] = robot.quaternion[1];
	quaternion[2] = robot.quaternion[2];
	quaternion[3] = robot.quaternion[3];
}


//}

