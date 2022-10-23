#ifndef NUM_DECTEC_ARMORDETECTOR_H
#define NUM_DECTEC_ARMORDETECTOR_H

#pragma once

#include <Eigen/Dense>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "DNN_detect.h"
#include <iostream>

#include "robot_state.h"

using namespace std;

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))

//灯条结构体
struct Light : public cv::RotatedRect     //灯条结构体
{
    Light();
    explicit Light(cv::RotatedRect &box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p,p+4,[](const cv::Point2f &a, const cv::Point2f &b) {return a.y<b.y;});
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;
        height = POINT_DIST(top, bottom);
        width = POINT_DIST(p[0], p[1]);
        angle = top.x < bottom.x ? box.angle : 90 + box.angle;
        if(fabs(bottom.x - top.x) < 0.01) angle = 90;
        //angle = atan2(fabs(centerI.y - centerJ.y),(centerI.x - centerJ.x));
    }


    int lightColor;
    cv::Point2f top;
    cv::Point2f bottom;
    double angle;
    double height;
    double width;


};

//装甲板结构体
struct Armor : public cv::RotatedRect    //装甲板结构体
{
    Armor()
    {
        confidence = 0;
        id = 0;
        type = SMALL;
        grade = 0;
    }
    explicit Armor(cv::RotatedRect &box) : cv::RotatedRect(box)
    {
        confidence = 0;
        id = 0;
        type = SMALL;
        grade = 0;
    }
    cv::Point2f armor_pt4[4]; //左下角开始逆时针
    double confidence;
    int id;  // 装甲板类别
    int grade;
    EnermyType type;  // 装甲板类型
    Eigen::Vector3d current_position;  // 当前的真实坐标
//    int area;  // 装甲板面积
};

//主类
class ArmorDetector:public robot_state
{
public:
    ArmorDetector(); //构造函数初始化

    vector<Armor> autoAim(const cv::Mat &src); //将最终目标的坐标转换到摄像头原大小的

    double cnt;

private:
    int binThresh;

    //light_judge_condition
    double light_max_angle;
    double light_min_hw_ratio;
    double light_max_hw_ratio;   // different distance and focus
    double light_min_area_ratio;   // RotatedRect / Rect
    double light_max_area_ratio;
    double light_area_max;


    //armor_judge_condition
    double armor_big_max_wh_ratio;
    double armor_big_min_wh_ratio;
    double armor_small_max_wh_ratio;
    double armor_small_min_wh_ratio;
    double armor_max_angle;
    double armor_height_offset;
    double armor_ij_min_ratio;
    double armor_ij_max_ratio;
    double armor_max_offset_angle;

    //armor_grade_condition
    double near_standard;
    int grade_standard;
    int height_standard;

    //armor_grade_project_ratio
    double id_grade_ratio;
    double height_grade_ratio;
    double near_grade_ratio;

    cv::Mat _src;  // 裁剪src后的ROI
    cv::Mat originSrc;
    cv::Mat _binary;
    std::vector<cv::Mat> temps;

    Armor lastArmor;

    std::vector<Light> candidateLights; // 筛选的灯条
    std::vector<Armor> candidateArmors; // 筛选的装甲板
    vector<Armor> finalArmors;
    Armor finalArmor;  // 最终装甲板

    void setImage(const cv::Mat &src); //对图像进行设置

    void findLights(); //找灯条获取候选匹配的灯条

    void matchLights(); //匹配灯条获取候选装甲板

    void chooseTarget(); //找出优先级最高的装甲板

    bool isLight(Light& light, std::vector<cv::Point> &cnt);

    bool conTain(cv::RotatedRect &match_rect,std::vector<Light> &Lights, size_t &i, size_t &j);

    int armorGrade(const Armor& checkArmor);

    void detectNum(Armor& armor);

    static inline void dnn_detect(cv::Mat frame, Armor& armor)// 调用该函数即可返回数字ID
    {
        return DNN_detect::net_forward(DNN_detect::img_processing(std::move(frame), TO_GRAY), DNN_detect::read_net(NET_PATH), armor.id, armor.confidence);
    }

    static inline bool makeRectSafe(cv::Rect & rect, cv::Size size){
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            // 如果发现矩形是空的，则返回false
            return false;
        return true;
    }

    static inline bool height_sort(Armor &candidate1,Armor &candidate2)
    {
        return candidate1.size.height > candidate2.size.height;
    }
};



#endif //NUM_DECTEC_ARMORDETECTOR_H
