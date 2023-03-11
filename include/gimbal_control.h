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
		explicit AngleSolve();

//private:
		
		double big_w;
		double big_h;
		double small_w;
		double small_h;
		
		double fly_time;
		
		std::string self_type;
		
		Eigen::Matrix<double,3,3> F_EGN;
		Eigen::Matrix<double,1,5> C_EGN;
		Eigen::Matrix<double,3,3> RotationMatrix_imu;
		Eigen::Matrix<double,3,3> RotationMatrix_cam2imu;
		Eigen::Vector3d center_offset_position;
		
		cv::Mat F_MAT;
		cv::Mat C_MAT;
		cv::Mat _src;
		
		Eigen::Vector3d cam2imu(Eigen::Vector3d &cam_pos);
		Eigen::Vector3d imu2cam(Eigen::Vector3d &imu_pos);
		Eigen::Vector3d pixel2imu(Armor &armor);
        Eigen::Vector3d pixel2imu2(cv::Point2f pp);
		cv::Point2f imu2pixel(Eigen::Vector3d &imu_pos);
		cv::Point2f cam2pixel(Eigen::Vector3d &cam_pos);
		
		Eigen::Vector3d pnpSolve(const cv::Point2f p[4], int type, int method = cv::SOLVEPNP_IPPE);
		
		Eigen::Vector3d gravitySolve(Eigen::Vector3d &Pos);//just consider gravity no air resistance consider
		
		Eigen::Vector3d airResistanceSolve(Eigen::Vector3d &imu_Pos);//consider gravity asn air resistance
		
		void yawPitchSolve(Eigen::Vector3d &Pos);
		
		double BulletModel(double &x, float &v, double &angle);
		
		double getFlyTime(Eigen::Vector3d &pos);

		Eigen::Matrix3d quaternionToRotationMatrix();
		
		Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
		
		Eigen::Matrix3d eulerAnglesToRotationMatrix2(Eigen::Vector3d &theta);
	};
	
//}
#endif //SHSIHI_ANGLESOLVE_HPP