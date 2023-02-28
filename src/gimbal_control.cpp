#include "gimbal_control.h"

//#define SHOW_PNP_RRECT

using namespace cv;
using namespace Eigen;

//namespace robot_detection{
AngleSolve::AngleSolve()
{
    cv::FileStorage fs("../other/control_data.yaml", cv::FileStorage::READ);

    fs["big_w"] >> big_w;
    fs["big_h"] >> big_h;
    fs["small_w"] >> small_w;
    fs["small_h"] >> small_h;
    fs["self_type"] >> self_type;

    fs[self_type]["F_MAT"] >> F_MAT;
    fs[self_type]["C_MAT"] >> C_MAT;
    cv::cv2eigen(F_MAT,F_EGN);
    cv::cv2eigen(C_MAT,C_EGN);

    cv::Mat temp;
    fs[self_type]["RotationMatrix_cam2imu"] >> temp;
    cv::cv2eigen(temp,RotationMatrix_cam2imu);

    fs[self_type]["CenterOffset_cam2imu"] >> temp;
    cv::cv2eigen(temp,CenterOffset_cam2imu);

    fs.release();
    fly_time = 0;
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

Eigen::Matrix3d AngleSolve::quaternionToRotationMatrix(float quaternion[4])
{
    Eigen::Matrix3d R_x;
    float w=quaternion[0],x=quaternion[1],y=quaternion[2],z=quaternion[3];
    R_x << 1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w,
            2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w,
            2*x*z-2*y*w, 2*y*z+2*w*x, 1-2*x*x-2*y*y;

    float roll = atan2(2*y*z + 2*w*x,1 - 2*x*x - 2*y*y)/CV_PI * 180.0f;
    float pitch = asin(2*w*y - 2*x*z)/CV_PI*180.0f;
    float yaw = atan2(2*x*y + 2*w*z, 1 - 2*y*y - 2*z*z)/CV_PI*180.0f;

    // 先确定串口数据，再改下面的坐标
    std::cout<<"----------[quaternion_euler]-----------"<<std::endl;
    std::cout<<"[roll:]   |"<<roll<<std::endl;
    std::cout<<"[pitch:]  |"<<pitch<<std::endl;
    std::cout<<"[yaw:]    |"<<yaw<<std::endl;
    std::cout<<"----------[get_from_euler]-----------"<<std::endl;
    std::cout<<"[get_roll:]     |"<<ab_roll<<std::endl;
    std::cout<<"[get_pitch:]    |"<<ab_pitch<<std::endl;
    std::cout<<"[get_yaw:]      |"<<ab_yaw<<std::endl;

    return R_x;
}

void AngleSolve::init(float r, float p, float y, float quat[4], float speed)
{
    ab_roll = r;
    ab_pitch = p;
    ab_yaw = y;
    bullet_speed = speed;

    // 这是旧的，用欧拉角的旋转矩阵
    // Eigen::Vector3d theta = {r,p,y};
    // RotationMatrix_imu = eulerAnglesToRotationMatrix(theta);

    // 现在使用四元素得出旋转矩阵
    RotationMatrix_imu = quaternionToRotationMatrix(quat);
}

Eigen::Vector3d AngleSolve::cam2imu(Vector3d cam_pos)
{
    // cam 2 imu(init),through test get, xyz-rpy
    Vector3d pos_tmp;

    // 左右手系，单靠旋转矩阵转换不了，与其矩阵运算不如直接赋值
    // pos_tmp = RotationMatrix_cam2imu * cam_pos;
    pos_tmp = {cam_pos[0],cam_pos[2],-cam_pos[1]};
    // std::cout<<"tmp_pos: "<<pos_tmp<<std::endl;

    Vector3d imu_pos;
    imu_pos += CenterOffset_cam2imu;
    imu_pos = RotationMatrix_imu * pos_tmp;
    // 加上两个坐标系的中心点的偏移量，先平移后旋转


    // std::cout<<"imu_pos: "<<imu_pos<<std::endl;
    return imu_pos;
}

Eigen::Vector3d AngleSolve::imu2cam(Vector3d imu_pos)
{
    Vector3d tmp_pos;
    tmp_pos = RotationMatrix_imu.inverse() * imu_pos;
    tmp_pos -= CenterOffset_cam2imu;

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
    return pixel_pos;
}

Eigen::Vector3d AngleSolve::pixel2imu(Armor &armor, int method)
{
    armor.camera_position = pixel2cam(armor,1);
    Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
    return imu_pos;
}

Eigen::Vector3d AngleSolve::pixel2cam(Armor &armor, int method)
{
    armor.camera_position = pnpSolve(armor.armor_pt4,armor.type,method);
    return armor.camera_position;
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
    // std::cout<<"fvgbhjkvghbj: "<< Pos.transpose()  <<std::endl;
    //at world coordinate system
    float y = -(float)Pos[2];
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
    return Vector3d(Pos[0],Pos[1],-y_temp);  // imu
}

Eigen::Vector3d AngleSolve::pnpSolve(Point2f *p, int type, int method = SOLVEPNP_IPPE)
{
    double w = type == SMALL ? small_w : big_w;
    double h = type == SMALL ? small_h : big_h;
    cv::Point2f lu, ld, ru, rd;
    std::vector<cv::Point3d> ps = {
            {-w / 2 , -h / 2, 0.},
            { w / 2 , -h / 2, 0.},
            { w / 2 ,  h / 2, 0.},
            {-w / 2 ,  h / 2, 0.}
    };

    std::vector<cv::Point2f> pu;
    pu.clear();
    pu.push_back(p[3]);
    pu.push_back(p[2]);
    pu.push_back(p[1]);
    pu.push_back(p[0]);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec/*, SOLVEPNP_IPPE*/);
    cv::cv2eigen(tvec, tv);

//    Mat R;
//    Rodrigues(rvec,R);
//
//    // offset++
    std::cout<<"distance:   "<<tv.norm()<<std::endl;

#ifdef SHOW_MEASURE_RRECT
    Mat pnp_check = _src.clone();
        Mat rv_mat;
        Eigen::Matrix<double,3,3> rv;
        cv::Rodrigues(rvec,rv_mat);
        cv::cv2eigen(rv_mat,rv);
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

Eigen::Vector3d AngleSolve::yawPitchSolve(Vector3d &Pos)
{
    Eigen::Vector3d rpy;
    rpy[2] = atan2(Pos[1],Pos[0]) / CV_PI*180.0;
    rpy[1] = atan2(Pos[2],Pos[0]) / CV_PI*180.0;
    rpy[0] = ab_roll;
    return rpy;
}

double AngleSolve::getFlyTime(Eigen::Vector3d &pos)
{
    return pos.norm() / bullet_speed;
}

Eigen::Vector3d AngleSolve::getAngle(Eigen::Vector3d predicted_position)
{
    Vector3d world_dropPosition;
    world_dropPosition = airResistanceSolve(predicted_position);//calculate gravity and air resistance
    Eigen::Vector3d rpy = yawPitchSolve(world_dropPosition);//get need yaw and pitch
    return rpy;
}

//}
