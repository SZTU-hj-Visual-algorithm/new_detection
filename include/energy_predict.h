#include "energy_state.h"
#include "gimbal_control.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
/*原始方程：spd = 0.913sin(1.942t) + 1.177
  预测方程：由欧拉法得出：xita(k)= F*xita(k-1) + Q
  观测方程：beita = H*xita + R
  F是对spd的积分
  H = 1

*/

//
//extern cv::Mat  quan_src;
//extern float quan_ab_pitch;
//extern float quan_ab_yaw;
//extern float quan_ab_roll;
//extern float quan_speed;

class energy_pre :public energy
{
	//一阶卡尔曼
	double F;//状态转移矩阵
	double H;//观测矩阵
	
	double Q;//预测误差
	
	double R;//观测误差
	double angle_k_1 = 0.1;
	
	double sigma;//协防差矩阵
	
	double K;//卡尔曼增益
	double measure_angle = 0;
	
	
	bool flip_angle;
	double angle_k;

	Eigen::Matrix3d F_EGN;
	Eigen::Matrix<double,1,5> C_EGN;

public:
    AngleSolve AS;

    cv::Point2f Aim_armor;
    cv::RotatedRect center_R;

	float E_pitch = 0;//打大符要转的pitch
	float E_yaw = 0;//yaw

	long int start_time = -1;
	long int last_time = -1;
    double t = 0;
    double dt = 0;
    cv::Point last_p;

    int direct = -1;
    int dir_count = 0;

    double w_std=0.106,h_std=0.106;
	
	double shoot_delay = 2.65911;
	
	cv::Point2f start_p = {-1,-1};
	cv::Point2f start_c = {-1,-1};
	
	double start_angle = 0;
	
	energy_inf target;

	energy_pre();

	void hit_reset();
	void reset();

	void get_direct(cv::Point2f &Aim_arrmor);


	cv::Point2f angle2_xy(cv::Point2f &now_xy, double pre_angle);//角度转xy坐标
	double measured(cv::Point2f& xy);//观测函数,就是观测当前距离开始转过的角度
	
	double predict(double n_time, double dt, bool pre_not, bool samll_energy);//预测步
	double correct(double measure);//更新步
	
	cv::Point2f gravity_finish(cv::Point2f &pp);

	bool energy_detect(cv::Mat &src, int color);//集合函数,给外部main函数调用用
	bool energy_predict_aim(long int time, bool small_energy);
	
//	bool cal_dela_angle();

	inline Eigen::Vector3d pc_to_pu(Eigen::Vector3d& pc , double& depth)
	{
		return F_EGN * pc / depth;
	}
	
	inline Eigen::Vector3d pu_to_pc(Eigen::Vector3d& pu , double& depth)
	{
		return F_EGN.inverse()*(pu * depth) ;//transpose求矩阵转置,inverse求矩阵的逆
	}


	double keep_pi(double angle);//保持在-pi～pi间
	//	double keep_pi_2(double angle);//保持在-pi～pi间
};


