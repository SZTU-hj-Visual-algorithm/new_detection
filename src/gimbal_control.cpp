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
    
	fs[self_type]["center_offset_position"] >> temp;
	cv::cv2eigen(temp,center_offset_position);

    fs.release();
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

    std::cout<<"----------[quaternion_euler]-----------"<<std::endl;
    std::cout<<"[roll:]   |"<<roll<<std::endl;
    std::cout<<"[pitch:]  |"<<pitch<<std::endl;
    std::cout<<"[yaw:]    |"<<yaw<<std::endl;
    std::cout<<"----------[get_from_weifeng_euler]-----------"<<std::endl;
    std::cout<<"[get_yaw:]     |"<<ab_yaw<<std::endl;
    std::cout<<"[get_pitch:]   |"<<ab_pitch<<std::endl;

    return R_x;
}

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

Eigen::Vector3d AngleSolve::cam2imu(Vector3d &cam_pos)
{
    //imu:roll pitch yaw
    Vector3d pos_tmp;
    if (self_type == "omni_infantry" ||
        self_type == "balance_infantry" ||
    	self_type == "hero")
	{
		pos_tmp = {cam_pos[0],cam_pos[2],-cam_pos[1]};
	}
	pos_tmp +=center_offset_position;
	Vector3d imu_pos;
	imu_pos = RotationMatrix_imu * pos_tmp;
//    std::cout<<"imu_pos: "<<imu_pos<<std::endl;
    return imu_pos;
}

Eigen::Vector3d AngleSolve::imu2cam(Vector3d &imu_pos)
{
    Vector3d tmp_pos;
    Vector3d cam_pos;
    tmp_pos = RotationMatrix_imu.inverse()*imu_pos;
	tmp_pos -=center_offset_position;
    if (self_type == "omni_infantry" ||
        self_type == "balance_infantry" ||
        self_type == "hero")
	{
		cam_pos = {tmp_pos[0],-tmp_pos[2],tmp_pos[1]};
	}
    return cam_pos;
}

Point2f AngleSolve::imu2pixel(Vector3d &imu_pos)
{
    Vector3d cam_pos = imu2cam(imu_pos);

    Point pixel_pos = cam2pixel(cam_pos);

    return pixel_pos;
}

Point2f AngleSolve::cam2pixel(Eigen::Vector3d &cam_pos)
{
    Vector3d tmp_pixel;
    tmp_pixel = F_EGN * cam_pos;
    // std::cout<<"tmp_pixel: "<<tmp_pixel<<std::endl;
    Point2f pixel_pos = {(float)(tmp_pixel[0]/tmp_pixel[2]),(float)(tmp_pixel[1]/tmp_pixel[2])};

    return pixel_pos;
}

Eigen::Vector3d AngleSolve::gravitySolve(Vector3d &Pos)
{
    //at world coordinate system
    double height;

    double del_ta = pow(bullet_speed, 4) + 2 * 9.8 * Pos(1, 0) * bullet_speed * bullet_speed - 9.8 * 9.8 * Pos(2, 0) * Pos(2, 0);
    double t_2 = (9.8 * Pos(1, 0) + bullet_speed * bullet_speed - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
    height = 0.5 * 9.8 * t_2;

    return Vector3d(Pos(0,0),Pos(1,0), Pos(2,0) + height);

}

Eigen::Vector3d AngleSolve::airResistanceSolve(Vector3d &imu_Pos)
{
    //at world coordinate system
    auto y = (double)imu_Pos(2,0);
    auto x = (double)sqrt(imu_Pos(0,0)*imu_Pos(0,0)+imu_Pos(1,0)*imu_Pos(1,0));
    double y_temp, y_actual, dy;
    double a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++)
    {
        a = (double)atan2(y_temp, x);
        y_actual = BulletModel(x, bullet_speed, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabs(dy) < 0.001) {
            break;
        }
//        printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/CV_PI,y_temp,dy);
    }

    return Vector3d(imu_Pos(0,0),imu_Pos(1,0),y_temp);
}

double AngleSolve::BulletModel(double &x, float &v, double &angle) { //x:m,v:m/s,angle:rad
    double y;
    fly_time = (double)((exp(SMALL_AIR_K * x) - 1) / (SMALL_AIR_K * v * cos(angle)));
    y = (double)(v * sin(angle) * fly_time - GRAVITY * fly_time * fly_time / 2);
    //printf("fly_time:%f\n",fly_time);
    return y;
}

Eigen::Vector3d AngleSolve::pnpSolve(const Point2f p[4], int type, int method)
{
    float w = type == SMALL ? small_w : big_w;
    float h = type == SMALL ? small_h : big_h;
    std::vector<cv::Point3d> ps = {
            {-w / 2 , -h / 2, 0.},
            {w / 2 , -h / 2, 0.},
            {w / 2 , h / 2, 0.},
            {-w / 2 , h / 2, 0.}
    };


    std::vector<cv::Point2f> pu;
    pu.push_back(p[3]);
    pu.push_back(p[2]);
    pu.push_back(p[1]);
    pu.push_back(p[0]);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec,false,method);
    cv::cv2eigen(tvec, tv);

#ifdef SHOW_PNP_RRECT
Mat pnp = _src.clone();
Mat rv_mat;
Eigen::Matrix<double,3,3> rv;
cv::Rodrigues(rvec,rv_mat);
cv::cv2eigen(rv_mat,rv);
//    std::cout<<"rv"<<rv<<std::endl;

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

circle(pnp,m_lu,3,Scalar(0,255,0),-1);
circle(pnp,m_ld,3,Scalar(255,255,0),-1);
circle(pnp,m_ru,3,Scalar(0,0,255),-1);
circle(pnp,m_rd,3,Scalar(0,255,255),-1);
line(pnp,m_lu,m_ld,Scalar(0,0,0),2);
line(pnp,m_ld,m_rd,Scalar(255,0,0),2);
line(pnp,m_rd,m_ru,Scalar(255,0,255),2);
line(pnp,m_ru,m_lu,Scalar(255,255,0),2);
imshow("pnp",pnp);
//    std::cout<<"m_lu:"<<m_lu<<std::endl;
//    std::cout<<"m_ld:"<<m_ld<<std::endl;
//    std::cout<<"m_ru:"<<m_ru<<std::endl;
//    std::cout<<"m_rd:"<<m_rd<<std::endl;
#endif

    return tv;
}

double AngleSolve::getFlyTime(Eigen::Vector3d &pos)
{
    return pos.norm() / bullet_speed;
}


Eigen::Vector3d AngleSolve::pixel2imu(Armor &armor)
{
    armor.camera_position = pnpSolve(armor.armor_pt4,armor.type);
    Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
    return imu_pos;
}

Eigen::Vector3d AngleSolve::pixel2imu2(Point2f pp)
{
//    armor.camera_position = pnpSolve(armor.armor_pt4,armor.type);
//    Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
//    return imu_pos;
}


//}
