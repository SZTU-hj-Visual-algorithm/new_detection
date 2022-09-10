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

struct Light : public cv::RotatedRect   //灯条结构体
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
        angle = 0;

        //judge condition
        double max_angle = 40.0;
        double min_hw_ratio = 3;
        double max_hw_ratio = 10;   // different distance and focus
        double min_area_ratio = 0.6;   // RotatedRect / Rect
        double max_area_ratio = 1.0;

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


struct Armor     //装甲板结构体
{
    Light l_Light;
    Light r_Light;
    int enermyId;
    cv::RotatedRect final_Rect;

};

struct aimInformation
{
    int classId;
    cv::RotatedRect final_Rect;
};


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

    int detectNum(cv::RotatedRect &final_rect);

    bool conTain(cv::RotatedRect &match_rect,std::vector<cv::RotatedRect> &Lights, size_t &i, size_t &j);

    void setImage(const cv::Mat &src);

    bool isLight(const Light& light);

    std::vector<Light> findLights();

    void matchLights();

    aimInformation chooseTarget();

private:
    cv::Mat _src;
    cv::Mat _binary;
    std::vector<cv::Mat> temps;
    bool Lost;
    int lostCnt;
    bool smallArmor;
    cv::Rect detectRoi;
    cv::RotatedRect lastArmor;
    std::vector<Light> candidateLights;
    std::vector<Armor> candidateArmors;
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

};



#endif //NUM_DECTEC_ARMORDETECTOR_H
