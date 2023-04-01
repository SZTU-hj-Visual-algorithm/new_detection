#include "gimbal_control.h"
#include <opencv2/core/eigen.hpp>

// #define SHOW_MEASURE_RRECT

using namespace cv;
using namespace Eigen;
AngleSolve::AngleSolve()
{
    cv::FileStorage fs("../other/control_data.yaml", cv::FileStorage::READ);

    big_w = (double)fs["big_w"];
    big_h = (double)fs["big_h"];
    small_w = (double)fs["small_w"];
    small_h = (double)fs["small_h"];
    buff_r_w = (double)fs["buff_r_w"];
    buff_r_h = (double)fs["buff_r_h"];
    buff_in_w = (double)fs["buff_in_w"];
    buff_in_h = (double)fs["buff_in_h"];
    buff_out_w = (double)fs["buff_out_w"];
    buff_out_h = (double)fs["buff_out_h"];
    buff_radius = (double)fs["buff_radius"];
    buff_convex = (double)fs["buff_convex"];

    fs["self_type"] >> self_type;

    fs[self_type]["F_MAT"] >> F_MAT;
    fs[self_type]["C_MAT"] >> C_MAT;
    cv::cv2eigen(F_MAT,F_EGN);
    cv::cv2eigen(C_MAT,C_EGN);

    cv::Mat temp;
    fs[self_type]["center_offset_position"] >> temp;
    cv::cv2eigen(temp,center_offset_position);

    fs[self_type]["gimbal_offset_angle"] >> temp;
    cv::cv2eigen(temp,gimbal_offset_angle);

    fs.release();
}


Eigen::Vector3d AngleSolve::pnpSolve(Point2f *p, int type)
{
    std::vector<cv::Point3d> ps;
    std::vector<cv::Point2f> pu;

    if(type == SMALL)
    {
        double w = small_w;
        double h = small_h;
        ps = {
                {-w / 2 , -h / 2, 0.},
                { w / 2 , -h / 2, 0.},
                { w / 2 ,  h / 2, 0.},
                {-w / 2 ,  h / 2, 0.}
        };
        pu.push_back(p[3]);
        pu.push_back(p[2]);
        pu.push_back(p[1]);
        pu.push_back(p[0]);
    }
    else if(type == BIG)
    {
        double w = big_w;
        double h = big_h;
        ps = {
                {-w / 2 , -h / 2, 0.},
                { w / 2 , -h / 2, 0.},
                { w / 2 ,  h / 2, 0.},
                {-w / 2 ,  h / 2, 0.}
        };
        pu.push_back(p[3]);
        pu.push_back(p[2]);
        pu.push_back(p[1]);
        pu.push_back(p[0]);
    }
    else if(type == BUFF_R)
    {
        double w = buff_r_w;
        double h = buff_r_h;
        ps = {
                {-w / 2 , -h / 2, 0.},
                { w / 2 , -h / 2, 0.},
                { w / 2 ,  h / 2, 0.},
                {-w / 2 ,  h / 2, 0.}
        };
        pu.push_back(p[0]);
        pu.push_back(p[1]);
        pu.push_back(p[2]);
        pu.push_back(p[3]);
    }
    else if(type == BUFF_NO)
    {
        ps = {
                {-buff_out_w / 2 , -buff_out_h , 0.},
                { buff_out_w / 2 , -buff_out_h , 0.},
                { buff_in_w / 2 ,  buff_in_h , 0.},
                {-buff_in_w / 2 ,  buff_in_h , 0.},
                {0 , buff_radius, -buff_convex},
        };
        pu.push_back(p[0]);
        pu.push_back(p[1]);
        pu.push_back(p[2]);
        pu.push_back(p[3]);
        pu.push_back(p[4]);
    }
    else if(type == BUFF_YES)
    {
        ps = {
                {-buff_out_w / 2 , -buff_out_h , 0.},
                { buff_out_w / 2 , -buff_out_h , 0.},
                { buff_in_w / 2 ,  buff_in_h , 0.},
                {-buff_in_w / 2 ,  buff_in_h , 0.},
                {0 , buff_radius, -buff_convex},
        };
        pu.push_back(p[0]);
        pu.push_back(p[1]);
        pu.push_back(p[2]);
        pu.push_back(p[3]);
        pu.push_back(p[4]);
    }

    cv::Mat rvec;
    cv::Mat tvec;

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec/*, SOLVEPNP_IPPE*/);

    Mat rv_mat;
    cv::Rodrigues(rvec,rv_mat);
    cv::cv2eigen(rv_mat,rv);
    cv::cv2eigen(tvec, tv);

#ifdef SHOW_MEASURE_RRECT
    Mat pnp_check = _src.clone();
        std::cout<<"rv"<<rv<<std::endl;

        Eigen::Vector3d imuPoint = {-w / 2 , -h / 2, 0.};
        Eigen::Vector3d armorPoint = rv*imuPoint + tv;//in camera coordinate
        cv::Point2f m_lu,m_ld,m_ru,m_rd;
        m_lu = cam2pixel(armorPoint);

        imuPoint = {-w / 2 , h / 2, 0.};
        armorPoint = rv*imuPoint + tv;
        m_ld = cam2pixel(armorPoint);

        imuPoint = {w / 2 , -h / 2, 0.};
        armorPoint = rv*imuPoint + tv;
        m_ru = cam2pixel(armorPoint);

        imuPoint = {w / 2 , h / 2, 0.};
        armorPoint = rv*imuPoint + tv;
        m_rd = cam2pixel(armorPoint);

        circle(pnp_check,m_lu,3,Scalar(0,255,0),-1);
        circle(pnp_check,m_ld,3,Scalar(255,255,0),-1);
        circle(pnp_check,m_ru,3,Scalar(0,0,255),-1);
        circle(pnp_check,m_rd,3,Scalar(0,255,255),-1);
        line(pnp_check,m_lu,m_ld,Scalar(0,0,0),2);
        line(pnp_check,m_ld,m_rd,Scalar(255,0,0),2);
        line(pnp_check,m_rd,m_ru,Scalar(255,0,255),2);
        line(pnp_check,m_ru,m_lu,Scalar(255,255,0),2);
        std::cout<<"m_lu:"<<m_lu<<std::endl;
        std::cout<<"m_ld:"<<m_ld<<std::endl;
        std::cout<<"m_ru:"<<m_ru<<std::endl;
        std::cout<<"m_rd:"<<m_rd<<std::endl;
        std::cout<<"tvec:"<<cam2pixel(tv)<<std::endl;

        imshow("pnp_check",pnp_check);
#endif

    return tv;
}

Eigen::Vector3d AngleSolve::cam2imu(Vector3d cam_pos)
{
    // cam 2 imu(init),through test get, xyz-rpy
    Vector3d pos_tmp;

    // 左右手系，单靠旋转矩阵转换不了，与其矩阵运算不如直接赋值
    // pos_tmp = RotationMatrix_cam2imu * cam_pos;
    pos_tmp = {cam_pos[0],cam_pos[2],-cam_pos[1]};
    // std::cout<<"cam_pos: "<<cam_pos.transpose()<<std::endl;

    Vector3d imu_pos;
    pos_tmp += center_offset_position;
    imu_pos = RotationMatrix_imu * pos_tmp;
    // 加上两个坐标系的中心点的偏移量，先旋转后平移

    // std::cout<<"imu_pos: "<<imu_pos<<std::endl;
    return imu_pos;
}

Eigen::Vector3d AngleSolve::imu2cam(Vector3d imu_pos)
{
    Vector3d tmp_pos;
    tmp_pos = RotationMatrix_imu.inverse() * imu_pos;
    tmp_pos -= center_offset_position;

    Vector3d cam_pos;
    // cam_pos = RotationMatrix_cam2imu.inverse() * tmp_pos;
    cam_pos = {tmp_pos[0],-tmp_pos[2],tmp_pos[1]};
    return cam_pos;
}

cv::Point2f AngleSolve::cam2pixel(Eigen::Vector3d cam_pos)
{
    Vector3d tmp_pixel;
    tmp_pixel = F_EGN * cam_pos;
//         std::cout<<"tmp_pixel: "<<tmp_pixel<<std::endl;
    cv::Point2f pixel_pos = Point2f((float)tmp_pixel[0]/tmp_pixel[2],(float)tmp_pixel[1]/tmp_pixel[2]);

    return pixel_pos;
}

cv::Point2f AngleSolve::imu2pixel(Vector3d imu_pos)
{
    Vector3d cam_pos = imu2cam(imu_pos);
    cv::Point2f pixel_pos = cam2pixel(cam_pos);
    // std::cout<<"[pixel_pos0]:"<<pixel_pos.x<<std::endl;
    // std::cout<<"[pixel_pos1]:"<<pixel_pos.y<<std::endl;
    return pixel_pos;
}

Eigen::Vector3d AngleSolve::pixel2imu(Armor &armor)
{
    armor.camera_position = pixel2cam(armor);
    Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
    return imu_pos;
}

Eigen::Vector3d AngleSolve::pixel2cam(Armor &armor)
{
    armor.camera_position = pnpSolve(armor.armor_pt4,armor.type);
    return armor.camera_position;
}

Eigen::Vector3d AngleSolve::pnp2imu(Eigen::Vector3d pos)
{
    Eigen::Vector3d camera_position = pnp2cam(pos);
    Eigen::Vector3d imu_pos = cam2imu(camera_position);
    return imu_pos;
}

Eigen::Vector3d AngleSolve::pnp2cam(Eigen::Vector3d pos)
{
    Eigen::Vector3d camera_position = rv*pos + tv;
    return camera_position;
}

// for buff
Eigen::Vector3d AngleSolve::pixel2cam(cv::Point2f *p, int type)
{
    Eigen::Vector3d cam_pos = pnpSolve(p,type);
    return cam_pos;
}

Eigen::Vector3d AngleSolve::pixel2imu(cv::Point2f *p, int type)
{
    Eigen::Vector3d cam_pos = pixel2cam(p,type);
    Eigen::Vector3d imu_pos = cam2imu(cam_pos);
    return imu_pos;
}

Eigen::Vector3d AngleSolve::imu2buff(Eigen::Vector3d imu_pos)
{
    Eigen::Vector3d buff_pos = RotationMatrix_imu2buff.inverse() * imu_pos;
    return buff_pos;
}

Eigen::Vector3d AngleSolve::buff2imu(Eigen::Vector3d buff_pos)
{
    Eigen::Vector3d imu_pos = RotationMatrix_imu2buff * buff_pos;
    return imu_pos;
}

float AngleSolve::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t,y;
    t = (float)((exp(SMALL_AIR_K * x) - 1) / (SMALL_AIR_K * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t/* * cos(ab_pitch)*/ / 2);   // 在相机坐标系下需要这个垂直夹角
    //printf("fly_time:  %f\n",t);
    return y;
}

Eigen::Vector3d AngleSolve::airResistanceSolve(Vector3d Pos)
{
    //at world coordinate system
    float y = (float)Pos[2];
    // -----------要水平距离的融合，否则计算的距离会少，在视野边缘处误差会大----------
    float x = (float)sqrt(Pos[0]*Pos[0]+ Pos[1]*Pos[1]);

    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++)
    {
        a = (float)atan2(y_temp, x);
        y_actual = BulletModel(x, bullet_speed, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        // printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,y_temp,dy);
    }

    // return Vector3d(Pos[0],-y_temp,Pos[2]);  // cam
    return Vector3d(Pos[0],Pos[1],y_temp);  // imu
}

Eigen::Vector3d AngleSolve::yawPitchSolve(Vector3d &Pos)
{

    Eigen::Vector3d rpy;
    rpy[2] = -atan2(Pos[0],Pos[1]) / CV_PI*180.0;
    rpy[1] = atan2(Pos[2],Pos[1]) / CV_PI*180.0;
    // rpy[1] = 0;
    rpy[0] = atan2(Pos[2],Pos[0]) / CV_PI*180.0;
    // rpy[2] = -atan2(Pos[2],Pos[0]) / CV_PI*180.0;
    // rpy[1] = -(atan2(Pos[1],Pos[0]) / CV_PI*180.0-90);
    // rpy[0] = ab_pitch;
    return rpy;
}


Eigen::Vector3d AngleSolve::getAngle(Eigen::Vector3d predicted_position)
{
    Vector3d world_dropPosition;
    world_dropPosition = airResistanceSolve(predicted_position);//calculate gravity and air resistance
    Eigen::Vector3d rpy = yawPitchSolve(world_dropPosition);//get need yaw and pitch

    rpy[0] += gimbal_offset_angle[0];
    rpy[1] += gimbal_offset_angle[1];
    rpy[2] += gimbal_offset_angle[2];

    return rpy;
}

// zyx
Eigen::Matrix3d AngleSolve::eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

// xyz,固定相机和IMU两个坐标系的转换
Eigen::Matrix3d AngleSolve::eulerAnglesToRotationMatrix2(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_x * R_y * R_z;
    return R;
}

Eigen::Matrix3d AngleSolve::quaternionToRotationMatrix()
{
    Eigen::Matrix3d R_x;
    float w=quaternion[0],x=quaternion[1],y=quaternion[2],z=quaternion[3];
    R_x << 1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w,
            2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w,
            2*x*z-2*y*w, 2*y*z+2*w*x, 1-2*x*x-2*y*y;

    float roll = atan2(2*y*z + 2*w*x,1 - 2*x*x - 2*y*y)/CV_PI * 180.0f;
    float pitch = asin(2*w*y - 2*x*z)/CV_PI*180.0f;
    float yaw = atan2(2*x*y + 2*w*z, 1 - 2*y*y - 2*z*z)/CV_PI*180.0f;

    // std::cout<<"----------[quaternion_euler]-----------"<<std::endl;
    // std::cout<<"[roll:]   |"<<roll<<std::endl;
    // std::cout<<"[pitch:]  |"<<pitch<<std::endl;
    // std::cout<<"[yaw:]    |"<<yaw<<std::endl;
    // std::cout<<"----------[get_from_euler]-----------"<<std::endl;
    // std::cout<<"[get_roll:]     |"<<ab_roll<<std::endl;
    // std::cout<<"[get_pitch:]    |"<<ab_pitch<<std::endl;
    // std::cout<<"[get_yaw:]      |"<<ab_yaw<<std::endl;

    return R_x;
}

bool AngleSolve::pointsInLine(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, double error, int type)
{
    if(type)
    {
        // 斜率
        if(fabs((p3.y - p1.y) * (p2.x - p1.x) - (p2.y - p1.y) * (p3.x - p1.x)) <= error)
            return true;
        else
            return false;
    }
    else
    {
        // 夹角
        if(fabs(atan2(p2.y - p1.y, p2.x - p1.x)/CV_PI*180.0 - atan2(p3.y - p1.y, p3.x - p1.x)/CV_PI*180.0) <= error)
            return true;
        else
            return false;
    }
}

bool AngleSolve::circleLeastFit(const std::vector<cv::Point2f> &points, double &center_x, double &center_y, double &radius)
{
    center_x = 0.0f;
    center_y = 0.0f;
    radius = 0.0f;
    if (points.size() < 3)
    {
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = points.size();
    for (int i = 0; i < N; i++)
    {
        double x = points[i].x;
        double y = points[i].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = std::sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}

// count IoU
double AngleSolve::countArmorIoU(Armor armor1, Armor armor2)
{
    double area1 = armor1.size.area();
    double area2 = armor2.size.area();

    std::vector<cv::Point2f> cross_points;
    cv::rotatedRectangleIntersection(armor1, armor2, cross_points);

    double area3 = cv::contourArea(cross_points);

    return (area3) / (area1 + area2 - area3);
}

double AngleSolve::getFlyTime(Eigen::Vector3d &pos)
{
    return pos.norm() / bullet_speed;
}