#include <Eigen/Dense>

class KalmanFilter {

public:
    Eigen::VectorXd x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Eigen::MatrixXd K;    // Kalman增益
    Eigen::MatrixXd F;    // 转移矩阵
    Eigen::MatrixXd H;    // 观测矩阵
    Eigen::MatrixXd Q;    // 预测过程噪声偏差的方差
    Eigen::MatrixXd R;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    Eigen::MatrixXd P;    // 估计误差协方差

    KalmanFilter() = default;

    void initial(Eigen::Vector3d position)
    {
        H << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0;

        R << 0.01,    0,    0,
                0, 0.01,    0,
                0,    0, 0.01;

        Q << 10,  0,  0,  0,  0,  0,
                0, 10,  0,  0,  0,  0,
                0,  0, 10,  0,  0,  0,
                0,  0,  0,  2,  0,  0,
                0,  0,  0,  0,  2,  0,
                0,  0,  0,  0,  0,  2;

        P << 1,  0,  0,  0,  0,  0,
                0, 1,  0,  0,  0,  0,
                0,  0, 1,  0,  0,  0,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1;

        x_k1 << position[0], position[1], position[2], 0, 0, 0;
    }

    void initial(Eigen::Vector3d position, Eigen::Vector3d speed)
    {
        H << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0;

        R << 0.01,    0,    0,
                0, 0.01,    0,
                0,    0, 0.01;

        Q << 10,  0,  0,  0,  0,  0,
                0, 10,  0,  0,  0,  0,
                0,  0, 10,  0,  0,  0,
                0,  0,  0,  2,  0,  0,
                0,  0,  0,  0,  2,  0,
                0,  0,  0,  0,  0,  2;

        P << 1,  0,  0,  0,  0,  0,
                0, 1,  0,  0,  0,  0,
                0,  0, 1,  0,  0,  0,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1;

        x_k1 << position[0], position[1], position[2], speed[0], speed[1], speed[2];
    }

    void setF(double t)
    {
        P << 1,  0,  0,  t,  0,  0,
                0, 1,  0,  0,  t,  0,
                0,  0, 1,  0,  0,  t,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1;
    }

    void setP(Eigen::MatrixXd P_last)
    {
        this->P = P_last;
    }

    Eigen::VectorXd update(Eigen::Vector3d z_k) {

        // 预测下一时刻的值
        Eigen::MatrixXd p_x_k = F * x_k1;   //x的先验估计由上一个时间点的后验估计值和输入信息给出

        //求协方差
        P = F * P * F.transpose() + Q;  //计算先验均方差 p(n|n-1)=F^2*p(n-1|n-1)+q

        //计算kalman增益
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + Q)

        //修正结果，即计算滤波值
        x_k1 = p_x_k + K * (z_k - H * p_x_k);  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))

        //更新后验估计
        P = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P;   //计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]

        return x_k1;
    }

};
