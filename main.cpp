#include "camera.h"
#include <opencv2/core/core_c.h>
#include "include/ArmorDetector.hpp"

using namespace cv;

int main()
{

    auto camera_warper = new Camera;
    ArmorDetector autoShoot;
    Armor autoTarget;
    Mat src;
    auto time_start = std::chrono::steady_clock::now();

    if (camera_warper->init())
    {
        camera_warper->record_start();
        while (waitKey(10)!=27)
        {
            camera_warper->read_frame_rgb();
            src = cv::cvarrToMat(camera_warper->ipiimage).clone();

            auto time_cap = std::chrono::steady_clock::now();

            int time_stamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count()); // 获取时间戳
            autoTarget = autoShoot.autoAim(src, time_stamp, time_cap);

//        waitKey(0);


            camera_warper->release_data();
        }
        camera_warper->~Camera();
        return 0;
    }
    else
    {
        printf("No camera!!");
        return 0;
    }

}
