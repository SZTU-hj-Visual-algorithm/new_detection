#ifndef NUM_DECTEC_ARMORDETECTOR_H
#define NUM_DECTEC_ARMORDETECTOR_H

#pragma once

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
    Armor();
    explicit Armor(cv::RotatedRect &box) : cv::RotatedRect(box)
    {

    }
    double light_height_rate;  // 左右灯条高度比
    int id;  // 装甲板类别
    int area;  // 装甲板面积
    EnermyType type;  // 装甲板类型
};

//主类
class ArmorDetector:public robot_state
{
public:
    ArmorDetector(); //构造函数初始化

    void setImage(const cv::Mat &src); //对图像进行设置

    void findLights(); //找灯条获取候选匹配的灯条

    void matchLights(); //匹配灯条获取候选装甲板

    void chooseTarget(); //找出优先级最高的装甲板

    void chooseTarget2(); //找出优先级最高的装甲板

    Armor transformPos(const cv::Mat &src); //将最终目标的坐标转换到摄像头原大小的

private:
    int lostCnt;
    int binThresh;

    //light_judge_condition
    double light_max_angle;
    double light_min_hw_ratio;
    double light_max_hw_ratio;   // different distance and focus
    double light_min_area_ratio;   // RotatedRect / Rect
    double light_max_area_ratio;


    //armor_judge_condition
    double armor_max_wh_ratio;
    double armor_min_wh_ratio;
    double armor_max_angle;
    double armor_height_offset;
    double armor_ij_min_ratio;
    double armor_ij_max_ratio;

    //armor_grade_condition
<<<<<<< Updated upstream
    double armor_standart_wh;
=======
    double big_wh_standard;
    double small_wh_standard;
    double near_standard;

    //armor_grade_project_ratio
    double id_grade_ratio;
>>>>>>> Stashed changes
    double wh_grade_ratio;
    double height_grade_ratio;
    double near_grade_ratio;
    double angle_grade_ratio;

    bool Lost;
    bool smallArmor;

    cv::Mat _src;  // 裁剪src后的ROI
    cv::Mat _binary;
    std::vector<cv::Mat> temps;

    cv::Rect detectRoi;  //为了把src通过roi变成_src

    Armor lastArmor;


    std::vector<Light> candidateLights; // 筛选的灯条
    std::vector<Armor> candidateArmors; // 筛选的装甲板
    Armor finalArmor;  // 最终装甲板
    cv::Rect finalRect;  // 最终框住装甲板旋转矩形的正矩形

    cv::Point2f dst_p[4] = {cv::Point2f(0,60),cv::Point2f(0,0),cv::Point2f(30,0),cv::Point2f(30,60)};

    int grade_standard;

    bool isLight(Light& light, std::vector<cv::Point> &cnt);

    int detectNum(cv::RotatedRect &f_rect);

    bool conTain(cv::RotatedRect &match_rect,std::vector<Light> &Lights, size_t &i, size_t &j);

    inline int whGrade(const double wh_ratio)
    {
        // 可以选择大装甲板，小装甲板w/h最大大约在2.45左右，大装甲板大约在4左右
        return wh_ratio/armor_standart_wh > 1 ? 100 : (wh_ratio/armor_standart_wh) * 100;
    }

    inline int heightGrade(const double armor_height, const double begin_height, const double end_height)
    {
        // 都和第一个装甲板高度比较？假设第一个得分为中值，对以后的给分上下波动
        // 可能会出现超过一百的情况，
        // 找一个好一点的标准值，下面改成了面积
        double hRotation = (armor_height - end_height) / (begin_height - end_height);
        return hRotation * 100;
    }

    inline int nearGrade(const Armor &checkArmor)
    {
        cv::Rect img_center_rect(_src.cols * 0.3, _src.rows * 0.3, _src.cols * 0.7, _src.rows * 0.7);
        Point2f vertice[4];
        checkArmor.points(vertice);
        if (img_center_rect.contains(checkArmor.center) &&
        img_center_rect.contains(vertice[0]) &&
        img_center_rect.contains(vertice[1]) &&
        img_center_rect.contains(vertice[2]) &&
        img_center_rect.contains(vertice[3]) )
        {
            return 100;
        }
        else
        {
            return 0;
        }
    }

    inline int angleGrade(const double armorAngle)
    {
        double angle_ratio = (90.0-armorAngle) / 90.0;
        return angle_ratio*100;
    }

    int armorGrade(const Armor& checkArmor)
    {
        // 看看裁判系统的通信机制，雷达的制作规范；

        // 选择用int截断double

        // 长宽比
        int wh_grade;
        double big_wh_standard = 3.5; // 4左右最佳，3.5以上比较正，具体再修改
        double small_wh_standard = 2; // 2.5左右最佳，2以上比较正，具体再修改
        double wh_ratio = checkArmor.size.height>checkArmor.size.width ? checkArmor.size.height / checkArmor.size.width : checkArmor.size.width > checkArmor.size.height;
        if(checkArmor.type == BIG)
        {
            wh_grade = wh_ratio/big_wh_standard > 1 ? 100 : (wh_ratio/big_wh_standard) * 100;
        }
        else
        {
            wh_grade = wh_ratio/small_wh_standard > 1 ? 100 : (wh_ratio/small_wh_standard) * 100;
        }

        // 最大装甲板，用面积，找一个标准值（固定距离（比如3/4米），装甲板大小（Armor.area）大约是多少，分大小装甲板）
        // 比标准大就是100，小就是做比例，，，，可能小的得出来的值会很小
        int area_grade;
        double big_area_standard = 2000; // 不知道，得实测
        double small_area_standard = 1000; // 不知道，得实测
        if(checkArmor.type == BIG)
        {
            area_grade = checkArmor.area/big_area_standard > 1 ? 100 : (checkArmor.area/big_area_standard) * 100;
        }
        else
        {
            area_grade = checkArmor.area/small_area_standard > 1 ? 100 : (checkArmor.area/small_area_standard) * 100;
        }

        // 靠近中心，与中心做距离，设定标准值，看图传和摄像头看到的画面的差异
        int near_grade;
        double near_standard = 500;
        double pts_distance = POINT_DIST(checkArmor.center, Point2f(_src.cols * 0.5, _src.rows * 0.5));
        near_grade = pts_distance/near_standard > 1 ? 100 : (pts_distance/near_standard) * 100;

        // 角度不歪
        int angle_grade;
        angle_grade = (90.0 - checkArmor.angle) / 90.0 * 100;

        // 下面的系数得详细调节；
        int final_grade = wh_grade    * 0.4 +
                          area_grade  * 0.3 +
                          near_grade  * 0.2 +
                          angle_grade * 0.1 ;
    }

    inline bool makeRectSafe(cv::Rect & rect, cv::Size size){
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

    static inline bool area_sort(std::vector<cv::Point> &cnt1,std::vector<cv::Point> &cnt2)
    {
        return cv::contourArea(cnt1) > cv::contourArea(cnt2);
    }

    static inline bool height_sort(Armor &candidate1,Armor &candidate2)
    {
        return candidate1.size.height > candidate2.size.height;
    }


};



#endif //NUM_DECTEC_ARMORDETECTOR_H
