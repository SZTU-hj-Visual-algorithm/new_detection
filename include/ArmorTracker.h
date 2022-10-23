#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "ArmorDetector.hpp"
#include "AngleSolve.hpp"
#include "KalmanFilter.h"

// 目的：通过读取进来的armors，筛选出同ID和上一帧的的装甲板，做跟踪

enum TrackerState {
    DETECTING,   // 还未开始跟踪，作为跟踪第一帧的切换，初始化好卡尔曼
    LOSING,      // 处于丢失状态
    TRACKING,    // 处于跟踪状态
} ;

class ArmorTracker
{
public:
    ArmorTracker();

    void setMyAngle(cv::Mat src, float pitch, float yaw, float roll, float SPEED);

    // 返回是否找到（初始化卡尔曼）和enemy_armor
    void selectEnemy(vector<Armor> find_armors);

    // 计算当前的真实坐标
    Eigen::Vector3d getRealPosition(Armor armor);

    // 预测未来的真实坐标
    void getPredictedPositionAndSpeed(clock_t start_time);

    // 计算抬枪角度
    headAngle finalResult(cv::Mat src, vector<Armor> find_armors,clock_t start_time);



private:
    cv::Mat _src;

    Armor enemy_armors;

    AngleSolve AS;

    //KalmanFilter KF;

    TrackerState tracker_state;  // 此时跟踪器的状态

    int tracking_id;  // 跟踪的敌方ID

    int lost_aim_cnt;  // 丢失目标计数

    int lost_threshold;

    int change_aim_cnt;

    int change_aim_threshold;

    bool isChangeSameID;  // 单个目标不用切换

    double new_old_threshold; // 新旧坐标的距离阈值

    double cur_pre_threshold; // 当前和预测的坐标点的距离阈值

    Eigen::Vector3d predicted_position;  // 预测的坐标，也是要发送给电控角度的坐标计算的角度

    Eigen::Vector3d predicted_speed;  // 预测得到的速度

};

//        // 本帧的真实坐标和预测的下一帧坐标
//        double cur_pre_distance = (predicted_position - find_armors[0].current_position).norm();
//        if(cur_pre_distance > cur_pre_threshold)
//        {
//            lost_aim_cnt++;
//            if(lost_aim_cnt > lost_threshold)
//            {
//                lost_aim_cnt = 0;
//                tracking_id = 0;
//                tracker_state = DETECTING;
//                enemy_armors.clear();
//            }
//            return ;
//        }
