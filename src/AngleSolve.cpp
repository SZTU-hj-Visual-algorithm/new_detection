//
// Created by 蓬蒿浪人 on 2022/10/6.
//

//
// Created by 蓬蒿浪人 on 2022/9/16.
//

#include "AngleSolve.hpp"
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace Eigen;

AngleSolve::AngleSolve()
{
    F_MAT=(Mat_<double>(3, 3) << 1554.52600, 0.000000000000, 630.01725, 0.000000000000, 1554.47451, 519.78242, 0.000000000000, 0.000000000000, 1.000000000000);
    C_MAT=(Mat_<double>(1, 5) << -0.08424, 0.16737, -0.00006, 0.00014, 0.00000);
}

Eigen::Vector3d AngleSolve::transformPos2_World(Vector3d &Pos)
{
    Mat camera2_tuoluo = (Mat_<double>(3,1) << CV_PI/2,0,0);
    Mat eular = (Mat_<double>(3,1) << ab_pitch/180*CV_PI,ab_yaw/180*CV_PI,ab_roll/180*CV_PI);
    Mat rotated_mat,coordinate_mat;
    Rodrigues(camera2_tuoluo,coordinate_mat);
    Rodrigues(eular,rotated_mat);

    cv2eigen(rotated_mat,rotated_matrix);
    cv2eigen(coordinate_mat,coordinate_maxtrix);

    return rotated_matrix*(coordinate_maxtrix*Pos);
}

Eigen::Vector3d AngleSolve::transformPos2_Camera(Eigen::Vector3d &Pos)
{
    return coordinate_maxtrix.inverse()*(rotated_matrix.inverse()*Pos);
}

Eigen::Vector3d AngleSolve::gravitySolve(Vector3d &Pos)
{
    //at world coordinate system
    double height;

    double del_ta = pow(SPEED, 4) + 2 * 9.8 * Pos(1, 0) * SPEED * SPEED - 9.8 * 9.8 * Pos(2, 0) * Pos(2, 0);
    double t_2 = (9.8 * Pos(1, 0) + SPEED * SPEED - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
    height = 0.5 * 9.8 * t_2;

    return Vector3d(Pos(0,0),Pos(1,0), Pos(2,0) + height);

}

Eigen::Vector3d AngleSolve::airResistanceSolve(Vector3d &Pos)
{
    //at world coordinate system
    auto y = (float)Pos(2,0);
    auto x = (float)sqrt(Pos(0,0)*Pos(0,0)+Pos(1,0)*Pos(1,0));
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++)
    {
        a = (float)atan2(y_temp, x);
        y_actual = BulletModel(x, SPEED, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,y_temp,dy);
    }

    return Vector3d(Pos(0,0),Pos(1,0), y_temp);
}

Eigen::Vector3d AngleSolve::pnpSolve(vector<Point2f> &p, EnermyType type, int method = SOLVEPNP_IPPE)
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

    vector<cv::Point2f> pu;
    pu.push_back(lu);
    pu.push_back(ru);
    pu.push_back(rd);
    pu.push_back(ld);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;


    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec,false,method);


    cv::cv2eigen(tvec, tv);
}

void AngleSolve::yawPitchSolve(Vector3d &Pos)
{
    send.yaw = atan2(Pos(0,0) ,
                     Pos(2,0)) / CV_PI*180.0 - ab_yaw;
    send.pitch = atan2(Pos(1,0) ,
                       sqrt(Pos(0,0)*Pos(0,0) + Pos(2,0)*Pos(2,0))) / CV_PI*180.0 - ab_pitch;
}

void AngleSolve::getAngle(Armor &aimArmor)
{
    ////sample////
    Vector3d po;
    po << 1,0,0;
    /////////////
    Vector3d aimPosition,worldPosition,world_dropPosition,camera_dropPosition;
    aimPosition = pnpSolve(aimArmor.pts_4,aimArmor.type);//use PnP to get aim position

    worldPosition = transformPos2_World(aimPosition);//transform aim position to world coordinate system

    //world_dropPosition = gravitySolve(worldPosition);//calculate gravity

    world_dropPosition = airResistanceSolve(worldPosition);//calculate gravity and air resistance

    camera_dropPosition = transformPos2_Camera(world_dropPosition);//transform position to camera coordinate system to get angle

    yawPitchSolve(camera_dropPosition);//get need yaw and pitch

    ////output result/////
    std::cout<<worldPosition[0]<<std::endl;
    std::cout<<worldPosition[1]<<std::endl;
    std::cout<<worldPosition[2]<<std::endl;
    /////////////////////

}