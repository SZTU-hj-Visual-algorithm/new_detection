#ifndef robot_status_H
#define robot_status_H
#include <opencv2/opencv.hpp>
#include <cmath>
#include <chrono>

//basic parameter
#define GRAVITY 9.78
#define SMALL_AIR_K 0.01903
#define BIG_AIR_K 0.00556
#define BIG_LIGHT_AIR_K 0.00530
#define SHOOT_DELAY 0.07

//namespace robot_detection {

using seconds_duration = std::chrono::duration<double>;
using milliseconds_duration = std::chrono::duration<double,std::milli>;
using microseconds_duration = std::chrono::duration<double,std::micro>;
using chrono_time = decltype(std::chrono::high_resolution_clock::now());

//robot basic classes
enum EnermyColor { RED = 1, BLUE = 2 };
enum EnermyType  { SMALL = 1, BIG = 2 };
enum EnermyState { RUN = 1, SPIN = 2};
enum SpinHeading { UNKNOWN, CLOCKWISE, COUNTER_CLOCKWISE };

//robot state information from electronic control group
class robot_state
{
public:
    //电控发来的角度和弹速
    float ab_pitch;
    float ab_yaw;
    float ab_roll;
    float bullet_speed;
    float quaternion[4];
//    int enemy_color;

    robot_state() = default;

    void clone(robot_state &robot);

    void updateData(const float *data, const float *quat);
//    void updateData(float *data, int color);
};

//}
#endif //robot_status_H