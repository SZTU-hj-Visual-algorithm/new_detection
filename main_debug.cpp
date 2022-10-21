//
// Created by 蓬蒿浪人 on 2022/10/10.
//
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.hpp"
#include "AngleSolve.hpp"
#include "camera.h"

using namespace cv;

int main()
{
//    auto camera_warrper = new Camera;
    RobotState robotState;
    ArmorDetector autoShoot(robotState);
    AngleSolve angleSolve(robotState);
    vector<Armor> autoTargets;
    Mat src;
    VideoCapture v("../material/record.avi");
    int lost_count = 0;
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
            autoTargets = autoShoot.autoAim(src);
//            angleSolve.getAngle(autoTargets[0]);
//            string yaw = "yaw:" + convertToString(angleSolve.send.yaw);
//            string pitch = "pitch:" + convertToString(angleSolve.send.pitch);
//            string angleInformation = yaw + pitch;
//            putText(src,angleInformation,Point(0,0),FONT_HERSHEY_COMPLEX,1,Scalar(0,255,0),2);
            imshow("src",src);
            if (!autoTargets.empty())
            {
                printf("---------------main get target!!!---------------\n");
            }
            else
            {
                lost_count++;
                printf("----------------no target\n---------------");
//                waitKey(0);
            }
            if (waitKey(10) == 27)
            {
//                camera_warrper->~Camera();
                break;
            }
        }
//    }


    printf("lost_count:%d\n",lost_count);


    return 0;
}

