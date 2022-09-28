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

Vector3d AngleSolve::transformPos2_World(Vector3d &Pos)
{
    Mat camera2_tuoluo = (Mat_<double>(3,1) << CV_PI/2,0,0);
    Mat eular = (Mat_<double>(3,1) << ab_pitch/180*CV_PI,ab_yaw/180*CV_PI,ab_roll/180*CV_PI);
    Mat rotated_mat,coordinate_mat;
    Rodrigues(camera2_tuoluo,coordinate_mat);
    Rodrigues(eular,rotated_mat);

    Matrix<double,3,3> rotated_matrix,coordinate_maxtrix;
    cv2eigen(rotated_mat,rotated_matrix);
    cv2eigen(coordinate_mat,coordinate_maxtrix);

    return rotated_matrix*(coordinate_maxtrix*Pos);
}


void AngleSolve::getAngle(Armor &aimArmor)
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

}

void AngleSolve::gravitySolve(Vector3d &Pos)
{
    double height;



}

void AngleSolve::pnpSolve(Point2f p[4], EnermyType type)
{
    double w = type == SMALL ? small_w : big_w;
    double h = type == SMALL ? small_h : big_h;
    cv::Point2f lu, ld, ru, rd;
    std::vector<cv::Point3d> ps = {
            {-w / 2 , -h / 2, 0.},
            {w / 2 , -h / 2, 0.},
            {w / 2 , h / 2, 0.},
            {-w / 2 , h / 2, 0.}
    };
    if (p[0].y < p[1].y) {
        lu = p[0];
        ld = p[1];
    }
    else {
        lu = p[1];
        ld = p[0];
    }
    if (p[2].y < p[3].y) {
        ru = p[2];
        rd = p[3];
    }
    else {
        ru = p[3];
        rd = p[2];
    }

    std::vector<cv::Point2f> pu;
    pu.push_back(lu);
    pu.push_back(ru);
    pu.push_back(rd);
    pu.push_back(ld);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;


    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec);


    cv::cv2eigen(tvec, tv);
}

void AngleSolve::yawPitchSolve(Eigen::Vector3d &Pos)
{
    send.yaw = atan2(Pos(0,0) ,
                     Pos(2,0)) / CV_PI*180.0 - ab_yaw;
    send.pitch = atan2(Pos(1,0) ,
                       sqrt(Pos(0,0)*Pos(0,0) + Pos(2,0)*Pos(2,0))) / CV_PI*180.0 - ab_pitch;
}
