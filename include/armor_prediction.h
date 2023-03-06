#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

//namespace robot_detection {

class KalmanFilter {

public:
    Eigen::Matrix<double, 6, 1> x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Eigen::Matrix<double, 6, 3> K;    // Kalman增益
    Eigen::Matrix<double, 6, 6> F;    // 转移矩阵
    Eigen::Matrix<double, 3, 6> H;    // 观测矩阵
    Eigen::Matrix<double, 6, 6> Q;    // 预测过程噪声偏差的方差
    Eigen::Matrix<double, 3, 3> R;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    Eigen::Matrix<double, 6, 6> P;    // 估计误差协方差

    // Priori error estimate covariance matrix
    Eigen::Matrix<double, 6, 6> P_pre;
    // Posteriori error estimate covariance matrix
    Eigen::Matrix<double, 6, 6> P_post;

    // Predicted state
    Eigen::VectorXd x_pre;
    // Updated state
    Eigen::Matrix<double, 6, 1> x_post;

    KalmanFilter() = default;

    void initial_KF();

    void setXPost(Eigen::Vector3d position);

    void setPosAndSpeed(Eigen::Vector3d position, Eigen::Vector3d speed);

    void setF(double t);

    void setP(Eigen::Matrix<double, 6, 6> P_last);

    Eigen::Matrix<double, 6, 1> predict();
    Eigen::Matrix<double, 6, 1> update(Eigen::Vector3d z_k);

};



//}