#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detection.h"
#include "armor_track.h"
#include "camera.h"
#include <stdlib.h>
#include <time.h>
using namespace cv;
using namespace robot_detection;

#define COLOR RED

int main()
{
    auto camera_warper = new Camera;
    double time = -1;
    float data[4] = {0,0,0,28};
    robot_state robot;
    robot.updateData(data,COLOR);
    ArmorDetector autoShoot;
    autoShoot.clone(robot);
    ArmorTracker autoTrack;
    autoTrack.AS.clone(robot);
    Skalman Singer;
    std::vector<Armor> autoTargets;
    Eigen::Vector3d predicted_position;
    Mat src;
    int lost_count = 0;
    if (camera_warper->init())
    {
        while(true)
        {
            if(camera_warper->read_frame_rgb())
            {
                //            	std::cout<<"init success"<<std::endl;
                src = camera_warper->src.clone();
                camera_warper->release_data();
            }

            autoTargets = autoShoot.autoAim(src);
            if (!autoTargets.empty())
            {
                Mat src_copy;
                printf("---------------main get target!!!---------------\n");
                double now_time = (double)getTickCount();
                src.copyTo(src_copy);
                if(autoTrack.locateEnemy(src_copy,autoTargets,now_time))
                {
                    //todo:show state
                    std::cout<<"track!!!"<<autoTrack.tracker_state<<"  id: "<<autoTrack.tracking_id<<std::endl;
                }
                else
                {
                    std::cout<<"loss!!!"<<std::endl;
                }
                //                sort(autoTargets.begin(),autoTargets.end(),
                //                     [](Armor &armor1,Armor &armor2){
                //                    return armor1.grade > armor2.grade;});
                //                double now_time = (double)getTickCount();
                //                if(time == -1)
                //                {
                //                    time = now_time;
                //                    continue;
                //                }
                //                double dt = (now_time - time) / (double)getTickFrequency();
                //                time = now_time;
                //                Eigen::Vector3d imuPos = autoTrack.pixel2imu(autoTargets[0]);
                //                circle(src,autoTargets[0].center,10,Scalar(255,0,0),-1);
                //                //                std::cout<<"imuPos:"<<imuPos<<std::endl;
                //                //                Eigen::Vector3d airPos = autoTrack.airResistanceSolve(imuPos);
                //                //                std::cout<<"airPos:"<<airPos<<std::endl;
                //                Eigen::Matrix<double,2,1> measure(imuPos(0,0),imuPos(1,0));
                //                std::cout<<measure<<std::endl;
                //                double all_time = SHOOT_DELAY + autoTrack.getFlyTime(imuPos);
                //                ////////////////Singer predictor//////////////////////////////
                //                Singer.PredictInit(dt);
                //                //                std::cout<<"dt:"<<dt<<std::endl;
                //                std::cout<<"predict_front:"<<Singer.predict(false)<<std::endl;
                //                std::cout<<"correct:"<<Singer.correct(measure)<<std::endl;
                //                Singer.PredictInit(all_time);
                //                Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
                //                std::cout<<"result:"<<predicted_result<<std::endl;
                //                predicted_position << predicted_result(0,0),predicted_result(3,0),imuPos(2,0);
                //                //                std::cout<<"predict_pos:"<<predicted_position<<std::endl;
                //                Eigen::Vector3d airPos = autoTrack.airResistanceSolve(predicted_position);
                //                cv::Point pixelPos = autoTrack.imu2pixel(airPos);
                //                ////////////////Singer predictor//////////////////////////////
                //                //                cv::Point pixelPos = autoTrack.imu2pixel(airPos);
                //                //                std::cout<<pixelPos<<std::endl;
                //                circle(src,pixelPos,10,Scalar(0,0,255),-1);
            }
            else
            {
                autoTrack.Singer.Reset();
                lost_count++;
                printf("----------------no target---------------\n");
            }
            imshow("src",src);
            if (waitKey(1) == 27)
            {
                camera_warper->~Camera();
                break;
            }
        }
    }
    printf("lost_count:%d\n",lost_count);
    return 0;
}
