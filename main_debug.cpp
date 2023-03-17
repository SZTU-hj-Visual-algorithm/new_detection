#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detection.h"
#include "armor_track.h"
#include "camera.h"
#include <stdlib.h>
#include <time.h>
//#include "serialport.h"
#include "serial_main.h"
using namespace cv;
//using namespace robot_detection;
SerialMain serial("/dev/robomaster");
pthread_mutex_t mutex_new;
pthread_cond_t cond_new;
pthread_mutex_t mutex_ka;
pthread_cond_t cond_ka;

bool is_ka = false;
bool is_start = false;
bool is_continue = true;

//#define COLOR RED

int main()
{
    auto camera_warper = new Camera;
//    float data[4] = {0,0,0,28};
    robot_state robot;
//    robot.updateData(data);
    ArmorDetector autoShoot;
//    autoShoot.clone(robot);
    ArmorTracker Track;
//    autoTrack.AS.clone(robot);
    Skalman Singer;
    std::vector<Armor> autoTargets;
    Eigen::Vector3d predicted_position;
    Mat src;
//    int lost_count = 0;
//    int mode_temp;
//    float lin[3];
//    float quat[4];
//    vision_t data;
//    float speed;
//    int color;
//    VisionData vdata;
//	port.initSerialPort();
    /*if (camera_warper->init())
    {*/
        while(true)
        {
            /*if(camera_warper->read_frame_rgb())
            {
                //            	std::cout<<"init success"<<std::endl;
                src = camera_warper->src.clone();
                camera_warper->release_data();
            }*/
            //!< 在每次角度变化的时候就不会缓冲区为空，bytes为22
            //!< 主要原因就是视觉这边只有接受程序的话，接受速率太快了导致中间很多帧收不到东西
//            bool dataGet = port.get_Mode1(mode_temp,lin[0],lin[1],lin[2],speed,color);
//            bool dataGet = port.get_Mode1_new(mode_temp, lin[0], lin[1], lin[2],quat);
			serial.ReceiverMain();
//			if(dataGet)std::cout<<"get"<<std::endl;
//			else std::cout<<"can't get"<<std::endl;
//			std::cout<<lin[0]<<std::endl;
			
//            Track.AS.updateData(lin,quat);
//            Eigen::Matrix3d quatRx = Track.AS.quaternionToRotationMatrix();
//			vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x31 };
//            port.TransformData(vdata);
//            port.send();


//            autoTargets = autoShoot.autoAim(src);
//            if (!autoTargets.empty())
//            {
//                Mat src_copy;
//                printf("---------------main get target!!!---------------\n");
//                double now_time = (double)getTickCount();
//                src.copyTo(src_copy);
//                if(autoTrack.locateEnemy(src_copy,autoTargets,now_time))
//                {
//                    std::cout<<"track!!!"<<autoTrack.tracker_state<<"  id: "<<autoTrack.tracking_id<<std::endl;
//                }
//                else
//                {
//                    std::cout<<"loss!!!"<<std::endl;
//                }
//            }
//            else
//            {
//                autoTrack.Singer.Reset();
//                lost_count++;
//                printf("----------------no target---------------\n");
//            }
            /*imshow("src",src);
            if (waitKey(1) == 27)
            {
                camera_warper->~Camera();
                break;
            }*/
        }
    //}
//    printf("lost_count:%d\n",lost_count);
    return 0;
}
