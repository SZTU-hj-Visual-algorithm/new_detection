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
    Mat eular = (Mat_<float>(3,1) << 0,1.57,0);
    Mat rotated_mat;
    Eigen::Matrix<float,3,3> rotated_matrix;
    Rodrigues(eular,rotated_mat);
    Eigen::Vector3d po;
    po << 1,0,0;
    cv2eigen(rotated_mat,rotated_matrix);
    Eigen::Vector3d dst_po = rotated_matrix*po;
    std::cout<<dst_po<< std::endl;
}