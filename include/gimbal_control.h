#ifndef SHSIHI_ANGLESOLVE_HPP
#define SHSIHI_ANGLESOLVE_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "armor_detection.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

//namespace robot_detection{

class AngleSolve : public robot_state
{
public:
    AngleSolve();

    void init(float r, float p, float y, float quat[4], float speed);

    Eigen::Vector3d getAngle(Eigen::Vector3d predicted_position);

//private:

    double big_w;
    double big_h;
    double small_w;
    double small_h;
    double buff_w;
    double buff_h;

    double fly_time;

    std::string self_type;

    Eigen::Matrix<double,3,3> RotationMatrix_cam2imu;
    Eigen::Vector3d CenterOffset_cam2imu;
    Eigen::Matrix<double,3,3> RotationMatrix_imu;

    cv::Mat F_MAT;
    cv::Mat C_MAT;
    Eigen::Matrix<double,3,3> F_EGN;
    Eigen::Matrix<double,1,5> C_EGN;
    Eigen::Matrix<double,3,3> rotated_matrix;
    Eigen::Matrix<double,3,3> coordinate_matrix;

    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
    Eigen::Matrix3d eulerAnglesToRotationMatrix2(Eigen::Vector3d &theta);
    Eigen::Matrix3d quaternionToRotationMatrix(float quaternion[4]);

    Eigen::Vector3d cam2imu(Eigen::Vector3d cam_pos);
    Eigen::Vector3d imu2cam(Eigen::Vector3d imu_pos);
    cv::Point2f cam2pixel(Eigen::Vector3d imu_pos);
    cv::Point2f imu2pixel(Eigen::Vector3d imu_pos);
    Eigen::Vector3d pixel2imu(Armor &armor, int method);
    Eigen::Vector3d pixel2cam(Armor &armor, int method);

    Eigen::Vector3d pnpSolve(cv::Point2f *p, int type, int method);

    Eigen::Vector3d airResistanceSolve(Eigen::Vector3d Pos);//consider gravity asn air resistance

    Eigen::Vector3d yawPitchSolve(Eigen::Vector3d &Pos);

    float BulletModel(float x, float v, float angle);

    double getFlyTime(Eigen::Vector3d &pos);
};

//}
#endif //SHSIHI_ANGLESOLVE_HPP