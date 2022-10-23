#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.hpp"
#include "ArmorTracker.h"
#include "camera.h"

using namespace cv;

int main()
{
//    auto camera_warrper = new Camera;
    ArmorDetector autoShoot;
    vector<Armor> autoTarget;
    Mat src;
    VideoCapture v("/home/lmx/new_detection/example.avi");
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
        clock_t start;
        start = clock();
        ArmorTracker autoTrack;
        headAngle sendAngle;
//            camera_warrper->read_frame_rgb(src);
        autoTarget = autoShoot.autoAim(src);

        sendAngle = autoTrack.finalResult(src,autoTarget, start);
        imshow("src",src);
        if (!autoTarget.empty())
        {
            printf("main get target!!!\n");
            printf("---------------main get target!!!---------------\n");
        }
        else
        {
            lost_count++;
            printf("----------------no target\n---------------");
            waitKey(0);
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

