//
// Created by 蓬蒿浪人 on 2022/9/16.
//

#ifndef SHSIHI_ANGLESOLVE_HPP
#define SHSIHI_ANGLESOLVE_HPP

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

    void getAngle(Eigen::Vector3d predicted_position);

//private:
    double big_w;
    double big_h;
    double small_w;
    double small_h;

    double fly_time;


    Eigen::Matrix<double,3,3> F_EGN;
    Eigen::Matrix<double,1,5> C_EGN;
    Eigen::Matrix<double,3,3> rotated_matrix;
    Eigen::Matrix<double,3,3> coordinate_maxtrix;

    cv::Mat F_MAT;
    cv::Mat C_MAT;

    headAngle send;

    Eigen::Vector3d transformPos2_World(Eigen::Vector3d &Pos);

    Eigen::Vector3d transformPos2_Camera(Eigen::Vector3d &Pos);

    Eigen::Vector3d pnpSolve(cv::Point2f *p, EnermyType type, int method);

    Eigen::Vector3d gravitySolve(Eigen::Vector3d &Pos);//just consider gravity no air resistance consider

    Eigen::Vector3d airResistanceSolve(Eigen::Vector3d &Pos);//consider gravity asn air resistance

    void yawPitchSolve(Eigen::Vector3d &Pos);

    float BulletModel(float x, float v, float angle);

    double getFlyTime();
};

#endif //SHSIHI_ANGLESOLVE_HPP