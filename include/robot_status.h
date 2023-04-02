#pragma once
//opencv
#include <opencv2/opencv.hpp>

//basic parameter
#define GRAVITY 9.78
#define SMALL_AIR_K 0.01903
#define BIG_AIR_K 0.00556
#define BIG_LIGHT_AIR_K 0.00530

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))
#define COLOR(str) std::strcmp(str.c_str(),"RED") == 0? RED : BLUE

//namespace robot_detection {

using seconds_duration = std::chrono::duration<double>;
using milliseconds_duration = std::chrono::duration<double,std::milli>;
using microseconds_duration = std::chrono::duration<double,std::micro>;
using chrono_time = decltype(std::chrono::high_resolution_clock::now());

//robot basic classes
enum EnemyColor { RED = 1, BLUE = 2 };
enum EnemyType  { SMALL = 1, BIG = 2, BUFF_R = 3, BUFF_NO = 4, BUFF_YES = 5};
enum EnemyState { RUN = 1, SPIN = 2};
enum SpinHeading { UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE };

//robot state information from electronic control group
class robot_state
{
public:
    //电控发来的角度和弹速
    float ab_pitch;
    float ab_yaw;
    float ab_roll;
    float quaternion[4];
    float bullet_speed;
    int enemy_color;

    robot_state() = default;

    void clone(robot_state &robot);
    void updateData(float data[4], float quat[4]);
    void updateDataColor(float data[4], float quat[4], int color);
};

//}
