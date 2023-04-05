#ifndef ARMOR_DETECTION_H
#define ARMOR_DETECTION_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "robot_status.h"
#include "number_DNN.h"
#include <iostream>

//namespace robot_detection {
    //灯条结构体
    struct Light : public cv::RotatedRect     //灯条结构体
    {
        Light() = default;
        explicit Light(cv::RotatedRect &box) : cv::RotatedRect(box)
        {
            cv::Point2f p[4];
            box.points(p);
            std::sort(p,p+4,[](const cv::Point2f &a, const cv::Point2f &b) {return a.y<b.y;});
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;
            height = POINT_DIST(top, bottom);
            width = POINT_DIST(p[0], p[1]);
            // sentry
//            angle = atan2(bottom.y-top.y, bottom.x-top.x) * 180 / CV_PI;
            // infantry
            angle = top.x < bottom.x ? box.angle : 90 + box.angle;
            if(fabs(bottom.x - top.x) <= 0.01) angle = 90;
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
        Armor() = default;
        explicit Armor(cv::RotatedRect &box) : cv::RotatedRect(box)
        {
            confidence = 0;
            id = 0;
            type = SMALL;
            grade = 0;
        }

         cv::Point2f armor_pt4[4]; //左下角开始逆时针
//        std::vector<cv::Point2f> armor_pt4; //左下角开始逆时针
        float confidence;
        int id;  // 装甲板类别
        int grade;
        int type;  // 装甲板类型
        Eigen::Vector3d world_position;  // 当前的真实坐标
        Eigen::Vector3d camera_position;  // 当前的真实坐标
//    int area;  // 装甲板面积
    };

//主类
    class ArmorDetector : public robot_state
    {
    public:
        ArmorDetector(); //构造函数初始化

        std::vector<Armor> autoAim(const cv::Mat &src); //将最终目标的坐标转换到摄像头原大小的

    private:
        int save_num_cnt;

        int binThresh;
        int enemy_color;
        int categories;

        // light_judge_condition
        double light_max_angle;
        double light_min_hw_ratio;
        double light_max_hw_ratio;   // different distance and focus
        double light_min_area_ratio;   // RotatedRect / Rect
        double light_max_area_ratio;
        double light_max_area;

        // armor_judge_condition
        double armor_big_max_wh_ratio;
        double armor_big_min_wh_ratio;
        double armor_small_max_wh_ratio;
        double armor_small_min_wh_ratio;
        double armor_max_angle;
        double armor_height_offset;
        double armor_ij_min_ratio;
        double armor_ij_max_ratio;
        double armor_max_offset_angle;

        // armor_grade_condition
        double near_standard;
        int grade_standard;
        int height_standard;

        // armor_grade_project_ratio
        double id_grade_ratio;
        double height_grade_ratio;
        double near_grade_ratio;

        float thresh_confidence;

        cv::Mat _src;  // 裁剪src后的ROI
        cv::Mat _binary;
        std::vector<cv::Mat> temps;

        Armor lastArmor;

        std::vector<Light> candidateLights; // 筛选的灯条
        std::vector<Armor> candidateArmors; // 筛选的装甲板
        std::vector<Armor> finalArmors;
        std::vector<cv::Mat> numROIs;
        Armor finalArmor;  // 最终装甲板

        DNN_detect dnnDetect;

        void setImage(const cv::Mat &src); //对图像进行设置

        void findLights(); //找灯条获取候选匹配的灯条

        void matchLights(); //匹配灯条获取候选装甲板

        void chooseTarget(); //找出优先级最高的装甲板

        bool isLight(Light& light, std::vector<cv::Point> &cnt);

        bool conTain(cv::RotatedRect &match_rect,std::vector<Light> &Lights, size_t &i, size_t &j);

        int armorGrade(const Armor& checkArmor);

        void preImplement(Armor& armor);

        bool get_max(const float *data, float &confidence, int &id);//serve single-task network

        bool get_valid(const float *data, float &confidence, int &id);// serve multi-task network
    };

//}

#endif //ARMOR_DETECTION_H
