//
// Created by 蓬蒿浪人 on 2022/10/10.
//
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.hpp"
#include "camera.h"

using namespace cv;

int main()
{
//    auto camera_warrper = new Camera;
    ArmorDetector autoShoot;
    vector<Armor> autoTarget;
    Mat src;
    VideoCapture v("../material/record.avi");
//    if (camera_warrper->init())
//    {
        while(true)
        {
            if (!v.isOpened()){
                break;
            }
            v.read(src);
            if (src.empty()){
                break;
            }
//            camera_warrper->read_frame_rgb(src);
            autoTarget = autoShoot.autoAim(src);
//            imshow("src",src);
            if (!autoTarget.empty())
            {
                printf("main get target!!!\n");
            }
            if (waitKey(10) == 27)
            {
//                camera_warrper->~Camera();
                break;
            }
        }
//    }





    return 0;
}

