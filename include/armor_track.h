#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
// #include "robot_status.h"
#include "armor_detection.h"
#include "armor_prediction.h"
#include "singer_prediction.h"
#include "gimbal_control.h"

// 目的：通过读取进来的armors，筛选出同ID和上一帧的的装甲板，做跟踪

//namespace robot_detection {

    enum TrackerState {
        MISSING,     // 没有目标，跳过该部分
        DETECTING,   // 还未开始跟踪，作为跟踪第一帧的切换，初始化好卡尔曼
        LOSING,      // 处于丢失状态，还会保留预测
        TRACKING,    // 处于跟踪状态
    };

    class ArmorTracker
    {
    public:
        cv::Mat _src;
        float pitch;
        float yaw;

        ArmorTracker();

        void reset();

        bool initial(std::vector<Armor> find_armors);

        bool selectEnemy(std::vector<Armor> find_armors, double dt);

        bool estimateEnemy(double dt);

        bool locateEnemy(cv::Mat src, std::vector<Armor> armors, const chrono_time &time);

        void show();

        Armor enemy_armor;
        Eigen::Vector3d bullet_point;

        AngleSolve AS;
        // Q和R分别代表对预测值和测量值的置信度（反比），通过影响卡尔曼增益K的值，影响预测值和测量值的权重。越大的R代表越不相信测量值。
        KalmanFilter KF;
        Skalman Singer;

        bool locate_target;

        int tracker_state;  // 此时跟踪器的状态
        int tracking_id;  // 跟踪的敌方ID

        int find_aim_cnt;
        int find_threshold;

        int lost_aim_cnt;  // 丢失目标计数
        int lost_threshold;

        double new_old_threshold; // 新旧坐标的距离阈值
        bool wait_start;
        chrono_time t;

        Eigen::Matrix<double,6,1> predicted_enemy;
        Eigen::Vector3d predicted_position;  // 预测的坐标，也是要发送给电控角度的坐标计算的角度
        Eigen::Vector3d predicted_speed;  // 预测得到的速度
    };

//}
