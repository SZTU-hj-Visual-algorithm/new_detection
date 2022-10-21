#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.hpp"
#include "ArmorTracker.h"
#include "camera.h"
#include <stdlib.h>
#include <time.h>
using namespace cv;

int main()
{
    auto camera_warrper = new Camera;
    ArmorDetector autoShoot;
    vector<Armor> autoTarget;

    ArmorTracker autoTrack;
    headAngle sendAngle;
    
    Mat src;
    if (camera_warrper->init())
    {
        while(true)
        {
            camera_warrper->read_frame_rgb(src);
            clock_t start;
            start = clock();
            autoTarget = autoShoot.autoAim(src);
            imshow("src",src);
            if (!autoTarget.empty())
            {
                printf("main get target!!!\n");
            }

            sendAngle = autoTrack.finalResult(autoTarget, start);

            if (waitKey(10) == 27)
            {
                camera_warrper->~Camera();
                break;
            }
        }
    }





    return 0;
}
