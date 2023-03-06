

#include "armor_prediction.h"

//namespace robot_detection {

void KalmanFilter::initial_KF()
{
    H << 1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0;

    cv::FileStorage fs("../other/predict_data.yaml", cv::FileStorage::READ);
    cv::Mat Q_, R_;
    fs["Q"] >> Q_;
    fs["R"] >> R_;
    fs.release();

    cv::cv2eigen(R_,R);
    cv::cv2eigen(Q_,Q);

    P <<1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
}

void KalmanFilter::setXPost(Eigen::Vector3d position)
{
    x_post << position[0], position[1], position[2], 0, 0, 0;
}

void KalmanFilter::setPosAndSpeed(Eigen::Vector3d position, Eigen::Vector3d speed)
{
    x_post << position[0], position[1], position[2], speed[0], speed[1], speed[2];
}

void KalmanFilter::setF(double t)
{
    F <<1, 0, 0, t, 0, 0,
    0, 1, 0, 0, t, 0,
    0, 0, 1, 0, 0, t,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
}

void KalmanFilter::setP(Eigen::Matrix<double, 6, 6> P_last)
{
    this->P = P_last;
}

Eigen::Matrix<double, 6, 1> KalmanFilter::predict()
{
    // 预测下一时刻的值
    x_pre = F * x_post;   //x的先验估计由上一个时间点的后验估计值和输入信息给出

    //求协方差
    P = F * P * F.transpose() + Q;  //计算先验均方差 p(n|n-1)=F^2*p(n-1|n-1)+q

    x_post = x_pre;

    return x_pre;
}

Eigen::Matrix<double, 6, 1> KalmanFilter::update(Eigen::Vector3d z_k) {
    //计算kalman增益
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + Q)

    //修正结果，即计算滤波值
    x_post = x_pre + K * (z_k - H * x_pre);  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))

    //更新后验估计
    P = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P;   //计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]

    return x_post;
}

//}