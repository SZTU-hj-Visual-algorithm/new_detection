//
// Created by 蓬蒿浪人 on 2022/9/16.
//

#ifndef SHSIHI_ANGLESOLVE_H
#define SHSIHI_ANGLESOLVE_H

#include "robot_state.h"
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include "ArmorDetector.hpp"

struct headAngle
{
    double yaw;
    double pitch;
};

class AngleSolve : public robot_state
{
public:
    AngleSolve();

    void getAngle(Armor &aimArmor);



private:
    double big_w;
    double big_h;
    double small_w;
    double small_h;

    Eigen::Matrix<double,3,3> F_EGN;
    Eigen::Matrix<double,1,5> C_EGN;

    cv::Mat F_MAT;
    cv::Mat C_MAT;

    headAngle send;

    Eigen::Vector3d transformPos2_World(Eigen::Vector3d &Pos);

    Eigen::Vector3d transformPos2_Camera(Eigen::Vector3d &Pos);

    void pnpSolve(Point2f p[4], EnermyType type);

    void gravitySolve(Eigen::Vector3d &Pos);

    void airResistanceSolve(Eigen::Vector3d &Pos);

    void yawPitchSolve(Eigen::Vector3d &Pos);




};

#endif //SHSIHI_ANGLESOLVE_H
