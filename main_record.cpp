//
// Created by 蓬蒿浪人 on 2022/10/10.
//
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detection.h"
#include "gimbal_control.h"
#include "armor_track.h"
#include "camera.h"

#define COLOR BLUE

using namespace cv;
//using namespace robot_detection;

int main()
{
//    auto camera_warrper = new Camera;
//    double time = -1;
    float data[3] = {0,0,28};
    float quat[4] = {0,0,0,0};
    robot_state robot;
    robot.updateData(data,quat);
    ArmorDetector autoShoot;
    autoShoot.clone(robot);
    ArmorTracker autoTrack;
    autoTrack.AS.clone(robot);
    std::vector<Armor> autoTargets;
    Eigen::Vector3d predicted_position;
    Mat src;
    VideoCapture v("../sample/Record-blue.avi");
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
//            robot_detection::headAngle sendAngle;
            auto start = std::chrono::high_resolution_clock::now();
            autoTargets = autoShoot.autoAim(src);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = seconds_duration(end-start).count();
            printf("run_time:%lf\n",duration);
//            angleSolve.getAngle(autoTargets[0]);
//            string yaw = "yaw:" + convertToString(angleSolve.send.yaw);
//            string pitch = "pitch:" + convertToString(angleSolve.send.pitch);
//            string angleInformation = yaw + pitch;
//            putText(src,angleInformation,Point(0,0),FONT_HERSHEY_COMPLEX,1,Scalar(0,255,0),2);
//            sendAngle = autoTrack.finalResult(src,autoTargets, start);
            if (!autoTargets.empty())
            {

                printf("---------------main get target!!!---------------\n");
//                sort(autoTargets.begin(),autoTargets.end(),
//                     [](Armor &armor1,Armor &armor2){
//                    return armor1.grade > armor2.grade;});
//                double now_time = (double)getTickCount();
				auto now_time = std::chrono::high_resolution_clock::now();
//                src.copyTo(autoTrack.AS._src);
                if(autoTrack.locateEnemy(src,autoTargets,now_time))
                {
                    //todo:show state
                    std::cout<<"track!!!"<<autoTrack.tracker_state<<"  id: "<<autoTrack.tracking_id<<std::endl;
                }
                else
                {
                    std::cout<<"loss!!!"<<std::endl;
                }
//                if(time == -1)
//                {
//                    time = now_time;
//                    continue;
//                }
//                double dt = (now_time - time) / (double)getTickFrequency();
//                time = now_time;
//                Eigen::Vector3d imuPos = autoTrack.pixel2imu(autoTargets[0]);
//                circle(src,autoTargets[0].center,5,Scalar(255,0,0),-1);
////                std::cout<<"imuPos:"<<imuPos<<std::endl;
////                Eigen::Vector3d airPos = autoTrack.airResistanceSolve(imuPos);
////                std::cout<<"airPos:"<<airPos<<std::endl;
//                Eigen::Matrix<double,2,1> measure(imuPos(0,0),imuPos(1,0));
////                std::cout<<measure<<std::endl;
//                double all_time = SHOOT_DELAY + autoTrack.getFlyTime(imuPos);
//                ////////////////Singer predictor//////////////////////////////
//                Singer.PredictInit(dt);
////                std::cout<<"dt:"<<dt<<std::endl;
//                /*std::cout<<"predict_front:"<<*/Singer.predict(false)/*<<std::endl*/;
//                /*std::cout<<"correct:"<<*/Singer.correct(measure)/*<<std::endl*/;
//                Singer.PredictInit(all_time);
//                Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
////                std::cout<<"result:"<<predicted_result<<std::endl;
//                predicted_position << predicted_result(0,0),predicted_result(3,0),imuPos(2,0);
////                std::cout<<"predict_pos:"<<predicted_position<<std::endl;
//                Eigen::Vector3d airPos = autoTrack.airResistanceSolve(predicted_position);
//                cv::Point pixelPos = autoTrack.imu2pixel(airPos);
//                ////////////////Singer predictor//////////////////////////////
////                cv::Point pixelPos = autoTrack.imu2pixel(airPos);
//                std::cout<<pixelPos<<std::endl;
//                circle(src,pixelPos,5,Scalar(0,0,255),-1);
//                imshow("src",src);
            }
            else
            {
                autoTrack.Singer.Reset();
                lost_count++;
                printf("----------------no target---------------\n");
//                waitKey(0);
            }
//            imshow("src",src);
            if (waitKey(1) == 27)
            {
//                camera_warrper->~Camera();
                break;
            }
        }
//    }


    printf("lost_count:%d\n",lost_count);


    return 0;
}

