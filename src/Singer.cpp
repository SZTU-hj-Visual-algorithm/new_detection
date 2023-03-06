#include "Singer.h"

Skalman::Skalman()
{

    //init H
    H << 1,0,0,0,0,0,
         0,0,0,1,0,0;

    //init R
    R << 0.00005, 0,
         0, 0.00005;

    //init Xk_1
    Xk_1 << 0,0.1,0,
            0,0.1,0;

    //init Xk
//    Xk = Xk_1;

    //init P
    double r11 = R(0,0);
    double r22 = R(1,1);
    Eigen::Matrix<double,3,3> p11,p22;
    p11 <<r11, r11/initT, r11/(initT*initT),
    r11/initT, (2*r11)/(initT*initT), (3*r11)/pow(initT,3),
    r11/(initT*initT), (3*r11)/pow(initT,3), (4*r11)/pow(initT,4);

    p22<<r22, r22/initT, r22/(initT*initT),
    r22/initT, (2*r22)/(initT*initT), (3*r22)/pow(initT,3),
    r22/(initT*initT), (3*r22)/pow(initT,3), (4*r22)/pow(initT,4);

    P<<p11,Eigen::Matrix<double,3,3>::Zero(),
    Eigen::Matrix<double,3,3>::Zero(),p22;

    //init Zk
    Zk << 0.1,0.1;

    //init lamda
    Vk = Zk - H*Xk_1;
    _Sk = 0.5*Vk*Vk.transpose();
    Sk = H*P*H.transpose() + R;
    lamda = std::max(1.,_Sk.trace()/Sk.trace());
    std::cout<<"lamda:"<<lamda<<std::endl;
}

void Skalman::Reset()
{
    //init Xk_1
    Xk_1 << 0,0.1,0,
            0,0.1,0;

    //init Xk
//    Xk = Xk_1;

    //init P
    double r11 = R(0,0);
    double r22 = R(1,1);
    Eigen::Matrix<double,3,3> p11,p22;
    p11 <<r11, r11/initT, r11/(initT*initT),
    r11/initT, (2*r11)/(initT*initT), (3*r11)/pow(initT,3),
    r11/(initT*initT), (3*r11)/pow(initT,3), (4*r11)/pow(initT,4);

    p22<<r22, r22/initT, r22/(initT*initT),
    r22/initT, (2*r22)/(initT*initT), (3*r22)/pow(initT,3),
    r22/(initT*initT), (3*r22)/pow(initT,3), (4*r22)/pow(initT,4);

    P<<p11,Eigen::Matrix<double,3,3>::Zero(),
    Eigen::Matrix<double,3,3>::Zero(),p22;

    //init Zk
    Zk << 0.1,0.1;
}


void Skalman::Reset(Eigen::Vector2d Xpos)
{
    //init Xk_1
    Xk_1 << Xpos(0,0),0.1,0,
            Xpos(1,0),0.1,0;

    //init Xk
//    Xk = Xk_1;

    //init P
    double r11 = R(0,0);
    double r22 = R(1,1);
    Eigen::Matrix<double,3,3> p11,p22;
    p11 <<r11, r11/initT, r11/(initT*initT),
    r11/initT, (2*r11)/(initT*initT), (3*r11)/pow(initT,3),
    r11/(initT*initT), (3*r11)/pow(initT,3), (4*r11)/pow(initT,4);

    p22<<r22, r22/initT, r22/(initT*initT),
    r22/initT, (2*r22)/(initT*initT), (3*r22)/pow(initT,3),
    r22/(initT*initT), (3*r22)/pow(initT,3), (4*r22)/pow(initT,4);

    P<<p11,Eigen::Matrix<double,3,3>::Zero(),
    Eigen::Matrix<double,3,3>::Zero(),p22;

    //init Zk
    Zk << 0.1,0.1;
}

void Skalman::setXpos(Eigen::Vector2d Xpos)
{
    Xk_1 << Xpos(0,0),0.1,0,
            Xpos(1,0),0.1,0;
}

void Skalman::PredictInit(const double &deleta_t)
{
    T = deleta_t;
//    std::cout<<"time_T:"<<(1-exp(-alefa*T))/alefa<<std::endl;

    //init F
    Eigen::Matrix<double,3,3> Fx;
    Eigen::Matrix<double,3,3> Fy;
    Fx<<1, T, (alefa*T-1+exp(-alefa*T))/(alefa*alefa),
    0, 1, (1-exp(-alefa*T))/alefa,
    0, 0, exp(-alefa*T);
    Fy = Fx;

    F << Fx,Eigen::Matrix<double,3,3>::Zero(),
    Eigen::Matrix<double,3,3>::Zero(),Fy;

    //init W
    Eigen::Matrix<double,3,3> Wx;
    Eigen::Matrix<double,3,3> Wy;
    double q11 = (2*pow(alefa,3)*pow(T,3) -
            6*pow(alefa,2)*pow(T,2) +
            6*alefa*T + 3 -
            12*alefa*T*exp(-alefa*T) -
            3*exp(-2*alefa*T))/
                    (6*pow(alefa,5));

    double q12 = (pow(alefa,2)*pow(T,2) -
            2*alefa*T + 1 -
            2*exp(-alefa*T) +
            exp(-2*alefa*T) +
            2*alefa*T*exp(-alefa*T))/
                    (2*pow(alefa,4));

    double q13 = (1 - 2*alefa*T*exp(-alefa*T) -
            exp(-2*alefa*T))/
                    (2*pow(alefa,3));

    double q21 = q12;

    double q22 = (2*alefa*T - 3 +
            4*exp(-alefa*T) -
            exp(-2*alefa*T))/
                    (2*pow(alefa,3));

    double q23 = (1 - 2*exp(-alefa*T) +
            exp(-2*alefa*T))/
                    (2*pow(alefa,2));

    double q31 = q13;

    double q32 = q23;

    double q33 = (1 - exp(-2*alefa*T))/(2*alefa);

    Wx << q11,q12,q13,
    q21,q22,q23,
    q31,q32,q33;
    Wy = Wx;

    W << 2*alefa*Sigmaq*Wx , Eigen::Matrix<double,3,3>::Zero(),
    Eigen::Matrix<double,3,3>::Zero() , 2*alefa*Sigmaq*Wy;

    //init G
    Eigen::Matrix<double,3,1> Gx,Gy;
    Gx << (-T + alefa*T*T/2 + (1-exp(-alefa*T))/alefa)/alefa,
    T - (1 - exp(-alefa*T))/alefa,
    1 - exp(-alefa*T);
    Gy = Gx;

    G<<Gx,Eigen::Matrix<double,3,1>::Zero(),
    Eigen::Matrix<double,3,1>::Zero(),Gy;

//    std::cout<<"F_Data:"<<F<<std::endl;
}

Eigen::Matrix<double,6,1> Skalman::predict(bool predict)
{
    _a << Xk(2,0), Xk(5,0);
    if(predict)
    {
        return F*Xk + G*_a;
    }
    Xk_1 = F*Xk + G*_a;
    //        std::cout<<"predictData:"<<Xk_1<<std::endl;
    return Xk_1;
}

Eigen::Matrix<double,6,1> Skalman::correct(Eigen::Matrix<double,2,1> &measure)
{
    Zk = measure;
    Vk = Zk - H*Xk_1;
    _Sk = (lamda*Vk*Vk.transpose())/(1+lamda);
    Sk = H*P*H.transpose() + R;
    lamda = std::max(1.,_Sk.trace()/Sk.trace());
    std::cout<<"lamda:"<<lamda<<std::endl;
    P = lamda*((F * P * F.transpose()) + W);
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    Xk = Xk_1 + K * (Zk - H * Xk_1);
    P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
    //        if((fabs(Xk(2,0) - Xk_1(2,0)) > axHold) ||
    //            (fabs(Xk(5,0) - Xk_1(5,0)) > ayHold))
    //        {
    //            Reset(measure);
    //        }
    //        std::cout<<"correct_Data:"<<Xk_1<<"-----------------------------"<<std::endl;
    return Xk_1;
}

bool Skalman::SingerPrediction(const double &dt,
                      const double &fly_time,
                      const Eigen::Matrix<double,3,1> &imu_position,
                      Eigen::Vector3d &predicted_position)
{
    Eigen::Matrix<double,2,1> measure(round(imu_position(0,0)*1000)/1000,
                                      round(imu_position(1,0)*1000)/1000);
    //std::cout<<"measureDepth:"<<measure[0]<<std::endl;
    //std::cout<<"measureY:"<<measure[1]<<std::endl;
    double all_time = SHOOT_DELAY + fly_time;
    PredictInit(dt);
    /*std::cout<<"predict_front:"<<*/predict(false)/*<<std::endl*/;
    /*std::cout<<"correct:"<<*/correct(measure)/*<<std::endl*/;
    PredictInit(all_time);
    Eigen::Matrix<double,6,1> predicted_result = predict(true);
    //std::cout<<"result:"<<predicted_result<<std::endl;
    predicted_position << predicted_result(0,0),
                        predicted_result(3,0),
                        imu_position(2,0);
    if (!predicted_position.norm()){
        return false;
    }
    return true;
}


