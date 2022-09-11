#ifndef NUM_DECTEC_ARMORDETECTOR_H
#define NUM_DECTEC_ARMORDETECTOR_H

#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/dnn/dnn.hpp"

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

        //judge condition
        max_angle = 40.0;
        min_hw_ratio = 3;
        max_hw_ratio = 10;      // different distance and focus
        min_area_ratio = 0.6;   // RotatedRect / Rect
        max_area_ratio = 1.0;
    }
    int lightColor;
    cv::Point2f top;
    cv::Point2f bottom;
    double angle;
    double height;
    double width;

    //judge condition
    double max_angle;
    double min_hw_ratio;
    double max_hw_ratio;   // different distance and focus
    double min_area_ratio;   // RotatedRect / Rect
    double max_area_ratio;
};

//装甲板结构体
struct Armor : public cv::RotatedRect    //装甲板结构体
{
    Armor();
    int enermyID;
};


//主类
class ArmorDetector:public robot_state
{
public:
    ArmorDetector()
    {
        lastArmor = cv::RotatedRect();
        detectRoi = cv::Rect();
        smallArmor = false;
        lostCnt = 0;
        Lost = true;
        cv::Mat temp1 = cv::imread("../template/1.png");
        cv::Mat temp3 = cv::imread("../template/3.png");
        cv::Mat temp4 = cv::imread("../template/4.png");
        cv::Mat temp2 = cv::imread("../template/2.png");
        cv::Mat temp6 = cv::imread("../template/6.png");
        cv::Mat temp8 = cv::imread("../template/8.png");

        cvtColor(temp1,temp1,cv::COLOR_BGR2GRAY);
        cvtColor(temp3,temp3,cv::COLOR_BGR2GRAY);
        cvtColor(temp4,temp4,cv::COLOR_BGR2GRAY);
        cvtColor(temp2,temp2,cv::COLOR_BGR2GRAY);
        cvtColor(temp6,temp6,cv::COLOR_BGR2GRAY);
        cvtColor(temp8,temp8,cv::COLOR_BGR2GRAY);

        temps.push_back(temp1);
        temps.push_back(temp3);
        temps.push_back(temp4);
        temps.push_back(temp2);
        temps.push_back(temp6);
        temps.push_back(temp8);
    }

    int detectNum(cv::RotatedRect &f_rect);

    bool conTain(Armor &match_rect,std::vector<Light> &Lights, size_t &i, size_t &j);

    void setImage(const cv::Mat &src);

    bool isLight(const Light& light);

    void findLights();

    void matchLights();

    void chooseTarget();

    Armor transformPos();

private:
    int lostCnt;
    const int binThresh = 150;

    bool Lost;
    bool smallArmor;

    cv::Mat _src;  //
    cv::Mat _binary;
    std::vector<cv::Mat> temps;
    cv::Rect detectRoi;
    cv::RotatedRect lastArmor;
    std::vector<Light> candidateLights; // 筛选的灯条
    std::vector<Armor> candidateArmors; // 筛选的装甲板
    Armor finalArmor;  // 最终装甲板
    cv::Rect finalRect;  // 最终框住装甲板旋转举行的正矩形
    cv::Point2f dst_p[4] = {cv::Point2f(0,60),cv::Point2f(0,0),cv::Point2f(30,0),cv::Point2f(30,60)};

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
};



#endif //NUM_DECTEC_ARMORDETECTOR_H
