#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "robot_status.h"
#include <cmath>
#define PI CV_PI

//namespace robot_detection {

    #define TANH2(x) (exp(2.5*x)-exp(-2.5*x))/(exp(2.5*x)+exp(-2.5*x))
    #define TANH_HALF(x) (exp(1.1*x)-exp(-1.1*x))/(exp(1.1*x)+exp(-1.1*x))

    //二维Singer模型
    class Skalman
    {
        double alefa = 1.0/30.0;//目标机动频率
        double Sigmaq = 0.01;//目标加速度标准差，Singer模型假设目标加速度符合均值为零的高斯分布
        double initT = 0.01;//用来初始化初始协方差矩阵的采样时间间隔（估算出来的）
        double axHold = 30;//x方向的加速度调整阈值
        double ayHold = 30;//y方向的加速度调整阈值
        double lamda;//渐消因子，减小滤波发散问题
        Eigen::Matrix<double, 6, 6> F;//状态转移矩阵
        Eigen::Matrix<double, 6, 6> W;//预测方程过程噪声
        Eigen::Matrix<double, 6, 1> Xk_1;//转移的状态
        Eigen::Matrix<double, 6, 1> Xk;//后验的状态
        Eigen::Matrix<double, 2, 1> Zk;//观测值
        Eigen::Matrix<double, 2, 6> H;//观测矩阵
        Eigen::Matrix<double, 6, 2> K;//卡尔曼增益
        Eigen::Matrix<double, 6, 6> P;//协方差矩阵
        Eigen::Matrix<double, 2, 2> R;//测量过程噪声
        Eigen::Matrix<double, 6, 2> G;//加速度控制矩阵
        Eigen::Matrix<double, 2, 1> _a;//平均加速度，传递加速度大致范围
        Eigen::Matrix<double, 2, 1> Vk;//新息序列，计算渐消因子需要
        Eigen::Matrix<double, 2, 2> _Sk;//根据测量值算出来的新息协方差
        Eigen::Matrix<double, 2, 2> Sk;//根据观测方程算出来的新息协方差
    public:
        double shoot_delay;
        double error_distance;

        double T = 0;//采样周期T，即前后两次预测帧相隔的时间
        double last_x[2] = {0,0};
        Eigen::Vector3d predicted_xyz = {0,0,0};

        Skalman();
        void Reset();
        void Reset(const Eigen::Vector2d &Xpos);
        void PredictInit(const double &deleta_t);
        void setXpos(const Eigen::Vector2d &Xpos);
        Eigen::Matrix<double,6,1> predict(bool predict);
        bool SingerPrediction(const double &dt,
                            const double &fly_time,
                            const Eigen::Matrix<double,3,1> &imu_position,
                            Eigen::Vector3d &predicted_position);

        Eigen::Matrix<double,6,1> correct(const Eigen::Matrix<double,2,1> &measure);
        double filter(const double &last, const double &current, const double &origin);
    };
//}