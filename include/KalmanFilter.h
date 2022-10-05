#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <Eigen/Dense>

template<int V_Z = 3, int V_X = 6>
class Kalman {
public:
    using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
    using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
    using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
    using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
    using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
    using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;
private:
    Matrix_x1d x_k1; // k-1ʱ�̵��˲�ֵ������k-1ʱ�̵�ֵ
    Matrix_xzd K;    // Kalman����
    Matrix_xxd F;    // ת�ƾ���
    Matrix_zxd H;    // �۲����
    Matrix_xxd Q;    // Ԥ���������ƫ��ķ���
    Matrix_zzd R;    // ��������ƫ�(ϵͳ����Ժ�ͨ������ͳ��ʵ����)
    Matrix_xxd P;    // �������Э����
    double last_t{ 0 };
public:
    Kalman() = default;

    Kalman() {
        t = 0.01;

        F << 1,  0,  0,  t,  0,  0,
             0,  1,  0,  0,  t,  0,
             0,  0,  1,  0,  0,  t,
             0,  0,  0,  1,  0,  0,
             0,  0,  0,  0,  1,  0,
             0,  0,  0,  0,  0,  1;

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
        
        x_k1 << 0, 0.0001, 0.001, 0, 0.0000001, 0.000001;

    }

    void reset(Matrix_xxd F, Matrix_zxd H, Matrix_xxd Q, Matrix_zzd R, Matrix_x1d init, double t) {
        this->F = F;
        this->H = H;
        this->P = Matrix_xxd::Zero();
        this->Q = Q;
        this->R = R;
        x_k1 = init;
        last_t = t;
    }

    void reset(Matrix_x1d init, double t) {
        x_k1 = init;
        last_t = t;
    }

    Matrix_x1d update(Matrix_z1d z_k, double t) {
        // ����ת�ƾ����е�ʱ����
        for (int i = 0; i < V_X; i++) {
            F(i + 3, i) = t - last_t;
        }
        last_t = t;

        // Ԥ����һʱ�̵�ֵ
        Matrix_x1d p_x_k = F * x_k1;   //x�������������һ��ʱ���ĺ������ֵ��������Ϣ����

        //��Э����
        P = F * P * F.transpose() + Q;  //������������� p(n|n-1)=F^2*p(n-1|n-1)+q

        //����kalman����
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();  //Kg(k)= P(k|k-1) H�� / (H P(k|k-1) H�� + Q)

        //����������������˲�ֵ
        x_k1 = p_x_k + K * (z_k - H * p_x_k);  //���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))

        //���º������
        P = (Matrix_xxd::Identity() - K * H) * P;   //������������  P[n|n]=(1-K[n]*H)*P[n|n-1]

        return x_k1;
    }

};

#endif /* _KALMAN_H_ */
