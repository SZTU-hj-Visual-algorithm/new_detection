#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "fmt/core.h"
#include "Spin_Tracker.h"
#include "armor_detection.h"
#include "singer_prediction.h"
#include "gimbal_control.h"
#include "armor_prediction.h"

// 目的：通过读取进来的armors，筛选出同ID和上一帧的的装甲板，做跟踪

//namespace robot_detection {

enum TrackerState {
    MISSING,     // 没有目标，跳过该部分
    DETECTING,   // 还未开始跟踪，作为跟踪第一帧的切换，初始化好卡尔曼
    LOSING,      // 处于丢失状态，还会保留预测
    TRACKING,    // 处于跟踪状态
};
struct Circle
{
    double x; // 圆心x坐标
    double y; // 圆心y坐标
    double r; // 半径
};
struct Jump_tracker {
    Jump_tracker() = default;
    chrono_time jump_time;
    Armor jump_armor;
};
struct Disappear_tracker{
    Disappear_tracker() = default;
    chrono_time disappear_time;
    Armor disappear_armor;
};


class ArmorTracker
{
public:
    int tracker_state;  // 此时跟踪器的状态
    int tracking_id;  // 跟踪的敌方ID

    float pitch;
    float yaw;
    float roll;

    Skalman Singer;

    AngleSolve AS;

    ArmorTracker();

    void reset();
    void show();
    bool initial(std::vector<Armor> find_armors);
    bool switchEnemy(std::vector<Armor> find_armors);
    bool selectEnemy(std::vector<Armor> find_armors, double dt);
    bool estimateEnemy(double dt);
    bool locateEnemy(const cv::Mat src, std::vector<Armor> armors, const chrono_time time, int mode);

    void update_jump_trackers(Armor new_armor);
    void reset_kf(Armor new_armor);
    void update_history_armors(Armor new_armor);
    void update_spin_T(Armor matched_armor, std::vector<Armor> &final_armors);
    bool updateSpinScore();
    void spin_detect();
    void save_spin_history();
    Armor enemy_armor;//最终选择的装甲板
    Armor real_armor; // virtual armor state, real armor
    KalmanFilter KF;

    int max_history_len;
    double last_r = 0.35;
    int vir_max = 20;
    int vir_num = 0;
    Eigen::Vector3d last_position;
    bool is_vir_armor = false;
    bool is_anti = false;
    bool is_target_move = false;
    int last_final_armors_size;
    double anti_spin_max_r_multiple;         // 符合陀螺条件，反陀螺分数增加倍数
    int anti_spin_judge_low_thres;           // 小于该阈值认为该车已关闭陀螺
    int anti_spin_judge_high_thres;          // 大于该阈值认为该车已开启陀螺
    int max_delta_t;                    //使用同一预测器的最大时间间隔(ms)
    double spin_T;
    Jump_tracker jump_tracker;
    Disappear_tracker disappear_tracker;
    std::vector<Jump_tracker> jump_trackers;

    std::map<int,int> new_armors_cnt_map;          //装甲板计数map，记录新增装甲板数
    std::multimap<int, SpinTracker> trackers_map;  //预测器Map
    std::map<int,SpinHeading> spin_status_map;     // 记录该车小陀螺状态（未知，顺时针，逆时针）
    std::map<int,double> spin_score_map;           // 记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
    std::deque<Armor> history_armors;
    std::deque<std::tuple<double, double, double>> history_spin_state;


    bool is_aim_virtual_armor;  // 出现虚拟装甲板后转过去

    bool locate_target;

    int find_aim_cnt;
    int find_threshold;

    int lost_aim_cnt;  // 丢失目标计数
    int lost_threshold;

    double new_old_threshold; // 新旧坐标的距离阈值

    bool wait_start;
    chrono_time t;

    cv::Mat _src;

    Eigen::Vector3d predicted_position;  // 预测的坐标，也是要发送给电控角度的坐标计算的角度
    Eigen::Vector3d predicted_speed;  // 预测得到的速度???
    Eigen::Matrix<double,6,1> predicted_enemy;
    Eigen::Vector3d bullet_point;


    int switch_armor_threshold;
    int switch_armor_cnt;

    // int switch_enemy_cnt;
    int switch_enemy_threshold;
    double max_effective_distance;

    bool is_switch_time_set;
    chrono_time last_change_time;
    double switch_time_threshold;

///---------------------switchEnemy---------------------
    int switch_enemy_cnt[5];
    bool flag[5];
    Armor temp[5];
    double delta_distance;
    double hero_distance;
//----------------------------------------------------------
};

//}