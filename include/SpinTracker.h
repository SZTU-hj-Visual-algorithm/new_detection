
#include <opencv2/opencv.hpp>
#include "robot_state.h"
#ifndef SHSIHI_ARMORTRACKER_H
#define SHSIHI_ARMORTRACKER_H

using namespace cv;
using namespace std;



//装甲板结构体
struct Armor : public cv::RotatedRect    //装甲板结构体
{
    Armor()
    {
        light_height_rate = 0;
        confidence = 0;
        id = 0;
        type = SMALL;
    }
    explicit Armor(cv::RotatedRect &box) : cv::RotatedRect(box)
    {
        light_height_rate = 0;
        confidence = 0;
        id = 0;
        type = SMALL;
    }
    double light_height_rate;  // 左右灯条高度比
    double confidence;
    int id;  // 装甲板类别
    EnermyType type;  // 装甲板类型
    int hit_score; // 击打分数
//    int area;  // 装甲板面积
};


class SpinTracker
{
public:
    Armor last_armor;                       //本次装甲板
    bool is_initialized;                    //是否完成初始化
    int last_timestamp;                     //本次装甲板时间戳
    const int max_history_len = 4;          //历史信息队列最大长度

    std::deque<Armor> history_info;  // 目标队列

    explicit SpinTracker(const Armor& src, int src_timestamp);
    bool update_tracker(const Armor& new_armor, int timestamp);
};
#endif //SHSIHI_ARMORTRACKER_H
