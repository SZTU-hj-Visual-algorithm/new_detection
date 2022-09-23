//
// Created by 蓬蒿浪人 on 2022/9/16.
//

#ifndef SHSIHI_ANGLESOLVE_H
#define SHSIHI_ANGLESOLVE_H

#include "robot_state.h"
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>

struct headAngle
{
    double yaw;
    double pitch;
};

class AngleSolve : public robot_state
{
public:
    AngleSolve();

    void getAngle(Eigen::Vector3d &pos);

private:
    Eigen::Matrix<double,3,3> F_EGN;
    Eigen::Matrix<double,1,5> C_EGN;
    cv::Mat F_MAT;
    cv::Mat C_MAT;

    double getGravity(Eigen::Vector3d &transedPos);

    headAngle send;
};

#endif //SHSIHI_ANGLESOLVE_H
