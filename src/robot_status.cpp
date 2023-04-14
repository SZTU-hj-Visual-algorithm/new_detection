#include "robot_status.h"

// 在ROS里，这些用不到
//namespace robot_detection {
        
    void robot_state::updateData(float data[4], float quat[4])
    {
        ab_roll  = data[2];
        ab_pitch = data[0];
        ab_yaw   = data[1];
        bullet_speed = data[3];
        quaternion[0] = quat[0];
	    quaternion[1] = quat[1];
	    quaternion[2] = quat[2];
	    quaternion[3] = quat[3];
        // printf("//////status//////\n");
//         printf("pitch :%f\n",ab_pitch);
//         printf("yaw   :%f\n",ab_yaw);
//         printf("roll  :%f\n",ab_roll);
//         printf("speed :%f\n",bullet_speed);
        // printf("////////////////\n");

    }

    void robot_state::updateDataColor(float data[4], float quat[4],int color)
    {
        updateData(data,quat);
        enemy_color = color;
        // printf("//////status//////\n");
        // printf("pitch :%f\n",ab_pitch);
        // printf("yaw   :%f\n",ab_yaw);
        // printf("roll  :%f\n",ab_roll);
        // printf("speed :%f\n",bullet_speed);
        // printf("color :%d\n",enemy_color);
        // printf("////////////////\n");
    }

    void robot_state::clone(robot_state &robot)
    {
        ab_roll = robot.ab_roll;
        ab_pitch = robot.ab_pitch;
        ab_yaw = robot.ab_yaw;
        bullet_speed = robot.bullet_speed;
        enemy_color = robot.enemy_color;
    }

//}