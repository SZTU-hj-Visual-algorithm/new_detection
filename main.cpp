#include <cstdio>
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
    auto time_start = std::chrono::steady_clock::now();
    if (camera_warrper->init())
    {
        while(true)
        {
            camera_warrper->read_frame_rgb(src);
            auto time_cap = std::chrono::steady_clock::now();

            int time_stamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count()); // 获取时间戳
            autoTarget = autoShoot.autoAim(src, time_stamp);
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
