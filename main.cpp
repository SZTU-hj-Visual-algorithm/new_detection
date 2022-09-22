#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.hpp"
#include "camera.h"

using namespace cv;

int main()
{
    auto camera_warrper = new Camera;
    ArmorDetector autoShoot;
    Armor autoTarget;
    Mat src;
    if (camera_warrper->init())
    {
        while(true)
        {
            camera_warrper->read_frame_rgb(src);
            autoTarget = autoShoot.autoAim(src);
            imshow("src",src);
            if (!autoTarget.size.empty())
            {
                printf("main get target!!!\n");
            }
            if (waitKey(10) == 27)
            {
                camera_warrper->~Camera();
                break;
            }
        }
    }





    return 0;
}
