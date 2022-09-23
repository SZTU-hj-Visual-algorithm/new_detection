//
// Created by 蓬蒿浪人 on 2022/9/16.
//

#include "AngleSolve.h"
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace Eigen;

AngleSolve::AngleSolve()
{
    F_MAT=(Mat_<double>(3, 3) << 1554.52600, 0.000000000000, 630.01725, 0.000000000000, 1554.47451, 519.78242, 0.000000000000, 0.000000000000, 1.000000000000);
    C_MAT=(Mat_<double>(1, 5) << -0.08424, 0.16737, -0.00006, 0.00014, 0.00000);
}

void AngleSolve::getAngle(Vector3d &pos)
{
    ////sample////
    Vector3d po;
    po << 1,0,0;
    /////////////

    Mat camera2_tuoluo = (Mat_<double>(3,1) << CV_PI/2,0,0);
    Mat eular = (Mat_<double>(3,1) << ab_pitch/180*CV_PI,ab_yaw/180*CV_PI,ab_roll/180*CV_PI);
    Mat rotated_mat,coordinate_mat;
    Rodrigues(camera2_tuoluo,coordinate_mat);
    Rodrigues(eular,rotated_mat);

    Matrix<double,3,3> rotated_matrix,coordinate_maxtrix;
    cv2eigen(rotated_mat,rotated_matrix);
    cv2eigen(coordinate_mat,coordinate_maxtrix);

    Vector3d dst_po = rotated_matrix*(coordinate_maxtrix*po);

    ////output result/////
    std::cout<<dst_po[0]<<std::endl;
    std::cout<<dst_po[1]<<std::endl;
    std::cout<<dst_po[2]<<std::endl;
    /////////////////////

    getGravity(dst_po);
}

double AngleSolve::getGravity(Eigen::Vector3d &transedPos)
{
    double height;



}