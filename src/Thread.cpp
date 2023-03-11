#include "thread.h"

using namespace cv;
using namespace std;

//SerialPort port;    // uart
SerialMain serial;  // usb

//one2tw
form _send_data;
Mat src;

//two2three
form send_data;
Mat ka_src;

void* Build_Src(void* PARAM)
{
    int lin_is_get/* = true*/;  // uart
    int mode_temp;              // uart
    float lin[3];               // uart
    float quat[4];              // uart

    int first_get/* = true*/;   // usb

    chrono_time time_temp;

    Mat get_src;
    auto camera_warper = new Camera;
    printf("camera_open-\n");

    if (camera_warper->init())
    {
        while (is_continue && (waitKey(1) != 27))
        {
            if (camera_warper->read_frame_rgb())
            {
                get_src = camera_warper->src.clone();

//				lin_is_get = port.get_Mode1_new(mode_temp, lin[0], lin[1], lin[2],quat);    // uart
                first_get = serial.ReceiverMain();                                          // usb

                time_temp = chrono::high_resolution_clock::now();
                pthread_mutex_lock(&mutex_new);
                {
                    get_src.copyTo(src);

                    // uart
//                    _send_data = {mode_temp,
//                                  lin_is_get,
//                                  {lin[0],lin[1],lin[2]},
//                                  {quat[0],quat[1],quat[2],quat[3]},
//                                  vector<Armor>(),
//                                  time_temp};
                    // usb
                    _send_data = {0x21,first_get,
                                  {serial.vision_msg_.pitch,
                                   serial.vision_msg_.yaw,
                                   serial.vision_msg_.shoot},
                                  {serial.vision_msg_.quaternion[0],
                                   serial.vision_msg_.quaternion[1],
                                   serial.vision_msg_.quaternion[2],
                                   serial.vision_msg_.quaternion[3]},
                                  vector<Armor>(),time_temp};

                    is_start = true;
                    pthread_cond_signal(&cond_new);
                    pthread_mutex_unlock(&mutex_new);
                    imshow("src",src);
                    camera_warper->release_data();
                }

            }
            else
            {
                src = cv::Mat();
            }
//			start = end;
        }
        camera_warper->~Camera();
        pthread_mutex_unlock(&mutex_new);
        is_continue = false;
    }
    else
    {
        printf("No camera!!\n");
        is_continue = false;
    }
}

void* Armor_Kal(void* PARAM)
{
    ArmorDetector Detect;
    std::vector<Armor> Targets;
    Mat src_copy;
    chrono_time time_temp;
    int mode_temp;
    int second_get;

    sleep(2);
    printf("Armor_open\n");
    while (is_continue)
    {
        pthread_mutex_lock(&mutex_new);

        while (!is_start) {

            pthread_cond_wait(&cond_new, &mutex_new);

        }
        is_start = false;

        src.copyTo(src_copy);
        Detect.updateData(_send_data.data,_send_data.quat);
        mode_temp = _send_data.mode;
        second_get = _send_data.dat_is_get;
        time_temp = _send_data.tim;
        pthread_mutex_unlock(&mutex_new);
        if(second_get)
        {
            printf("[mode_temp]:    |%x\n",mode_temp);
            if (mode_temp == 0x21)
            {
                Targets = Detect.autoAim(src_copy);
                if(!Targets.empty())std::cout<<"------------[Get Targets]--------------"<<std::endl;
                else 				std::cout<<"------------[No Targets]---------------"<<std::endl;
                pthread_mutex_lock(&mutex_ka);
                send_data = {mode_temp,
                             second_get,
                             {Detect.ab_pitch,Detect.ab_yaw,Detect.bullet_speed},
                             {Detect.quaternion[0],Detect.quaternion[1],Detect.quaternion[2],Detect.quaternion[3]},
                             Targets,
                             time_temp};
                src_copy.copyTo(ka_src);
                is_ka = true;
                pthread_cond_signal(&cond_ka);
                pthread_mutex_unlock(&mutex_ka);
            }
        }
    }
}

void* Kal_predict(void* PARAM)
{
    vector<double> vdata(3);    // usb
//    VisionData vdata;              // uart

    vector<Armor> armors;
    ArmorTracker Track;
    int mode_temp;
    int angle_get;
    chrono_time time_temp;
    Mat src_copy;

    long int time_count = 0;
    energy_pre E_predicter;

    sleep(3);
    printf("kal_open\n");
    while (is_continue)
    {
        pthread_mutex_lock(&mutex_ka);

        while (!is_ka) {

            pthread_cond_wait(&cond_ka, &mutex_ka);
        }

        is_ka = false;

        ka_src.copyTo(src_copy);
        Track.AS.updateData(send_data.data,send_data.quat);
        angle_get = send_data.dat_is_get;
        mode_temp = send_data.mode;
        armors = send_data.armors;
        time_temp = send_data.tim;
        pthread_mutex_unlock(&mutex_ka);
        if(angle_get)
        {
            if (mode_temp == 0x21)
            {
                Track.AS.bullet_speed = 28.0;
                if (Track.locateEnemy(src_copy,armors,time_temp))
                {
                    vdata = { Track.pitch, Track.yaw, 0x31 };
                }
                else
                {
                    //    原数据，无自瞄
                    vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x32 };
                }
                Track.show();
                // uart
//                port.TransformData(vdata);
//                port.send();
                // usb
                serial.SenderMain(vdata);
            } // else if (mode_temp == 0x21)
//            else if (mode_temp == 0x22)
//            {
//                bool small_energy = false;
//                //printf("big energy!!\n");
//                ka_src.copyTo(src_copy);
////                quan_ab_pitch = lin[0];
////                quan_ab_yaw = lin[1];
////                E_predicter.ab_roll = lin[2];
////                E_predicter.SPEED = lin[3];
////                send_data.is_get = lin_is_get;
//                time_count = getTickCount();
//
//                if (E_predicter.energy_detect(src, BLUE))
//                {
//                    if (E_predicter.energy_predict_aim(time_count,small_energy))
//                    {
//                        float p = E_predicter.E_pitch - E_predicter.AS.ab_pitch;
//                        float y = E_predicter.E_yaw - E_predicter.AS.ab_yaw;
//                        printf("de_yaw:%f     de_pitch:%f\n",-p,-y);
//
//                        vdata = { -p, -y, 0x31 };
//                    }
//                    else
//                    {
//                        vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x32 };
//                    }
//                }
//                else
//                {
//                    vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x32 };//---------
//                }
//            }  // else if (mode_temp == 0x22)
//            else if (mode_temp == 0x23)
//            {
//                bool small_energy = true;
//                //printf("samll energy!!\n");
//                ka_src.copyTo(src_copy);
////                quan_ab_pitch = lin[0];
////                quan_ab_yaw = lin[1];
////                E_predicter.ab_roll = lin[2];
////                E_predicter.SPEED = lin[3];
////                send_data.is_get = lin_is_get;
//                time_count = getTickCount();
//                //printf("quan_pitch:%f",quan_ab_pitch);
//                //printf("quan_yaw:%f",quan_ab_yaw);
//                if (E_predicter.energy_detect(src, BLUE)) //
//                {
//                    if (E_predicter.energy_predict_aim(time_count,small_energy))
//                    {
//                        float p = E_predicter.E_pitch - E_predicter.AS.ab_pitch;
//                        float y = E_predicter.E_yaw - E_predicter.AS.ab_yaw;
//                        printf("de_yaw:%f     de_pitch:%f\n",-p,-y);
//
//                        vdata = { -p, -y, 0x31 };//---------
//                    }
//                    else
//                    {
//                        vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x32 };//--------
//                    }
//                }
//                else
//                {
//                    vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x32 };//---------
//                }
//            } // else if (mode_temp == 0x23)

            // uart
//                port.TransformData(vdata);
//                port.send();
            // usb
            serial.SenderMain(vdata);

        }

    }
}

//////





//----------------------uart port------------
//#include "thread.h"
//#include <cstdio>
//#include <opencv2/opencv.hpp>
//#include <chrono>
//
//using namespace cv;
//using namespace std;
//
//SerialPort port;
//
////one2two
//form _send_data;
//Mat src;
//
////two2three
//form send_data;
//Mat ka_src;
//
//void* Build_Src(void* PARAM)
//{
//    int lin_is_get;
//    int mode_temp;
//    float lin[3];
//    float quat[4];
//    chrono_time time_temp;
//    Mat get_src;
//    auto camera_warper = new Camera;
//    port.initSerialPort();
//    printf("camera_open-\n");
//
//    if (camera_warper->init())
//    {
//        printf("1-real\n");
//        while (is_continue && (waitKey(1) != 27))
//        {
//            if (camera_warper->read_frame_rgb())
//            {
//                get_src = camera_warper->src.clone();
//
//                lin_is_get = port.get_Mode1_new(mode_temp, lin[0], lin[1], lin[2],quat);
//                time_temp = chrono::high_resolution_clock::now();
//                pthread_mutex_lock(&mutex_new);
//                {
//                    get_src.copyTo(src);
//                    _send_data = {mode_temp,
//                                  lin_is_get,
//                                  {lin[0],lin[1],lin[2]},
//                                  {quat[0],quat[1],quat[2],quat[3]},
//                                  vector<Armor>(),
//                                  time_temp};
//                    is_start = true;
//                    pthread_cond_signal(&cond_new);
//                    pthread_mutex_unlock(&mutex_new);
//                    imshow("src",src);
//                    camera_warper->release_data();
//                }
//            }
//            else
//            {
//                src = cv::Mat();
//            }
//        }
//        camera_warper->~Camera();
//        pthread_mutex_unlock(&mutex_new);
//        is_continue = false;
//    }
//    else
//    {
//        printf("No camera!!\n");
//        is_continue = false;
//    }
//}
//
//void* Armor_Kal(void* PARAM)
//{
//    ArmorDetector Detect;
//    std::vector<Armor> Targets;
//    Mat src_copy;
//    chrono_time time_temp;
//    int mode_temp;
//    int color_get;
//
//    sleep(2);
//    printf("Armor_open\n");
//    while (is_continue)
//    {
//        pthread_mutex_lock(&mutex_new);
//
//        while (!is_start) {
//            pthread_cond_wait(&cond_new, &mutex_new);
//        }
//        is_start = false;
//
//        src.copyTo(src_copy);
//        Detect.updateData(_send_data.data,_send_data.quat);
//        mode_temp = _send_data.mode;
//        color_get = _send_data.dat_is_get;
//        time_temp = _send_data.tim;
//
//        pthread_mutex_unlock(&mutex_new);
//        if(color_get)
//        {
//            if (mode_temp == 0x21)
//            {
//                Targets = Detect.autoAim(src_copy);
//                if(!Targets.empty())
//                    std::cout<<"------------Get Targets--------------"<<std::endl;
//                else
//                    std::cout<<"------------No Targets--------------"<<std::endl;
//                pthread_mutex_lock(&mutex_ka);
//                send_data = {mode_temp,
//                             color_get,
//                             {Detect.ab_pitch,Detect.ab_yaw,Detect.bullet_speed},
//                             {Detect.quaternion[0],Detect.quaternion[1],Detect.quaternion[2],Detect.quaternion[3]},
//                             Targets,
//                             time_temp};
//                src_copy.copyTo(ka_src);
//                is_ka = true;
//                pthread_cond_signal(&cond_ka);
//                pthread_mutex_unlock(&mutex_ka);
//            }
//        }
//
//
//    }
//}
//
//void* Kal_predict(void* PARAM)
//{
//    VisionData vdata;
//    vector<Armor> armors;
//    ArmorTracker Track;
//    int mode_temp;
//    int angle_get;
//    chrono_time time_temp;
//    Mat src_copy;
//
//    sleep(3);
//    printf("kal_open\n");
//    while (is_continue)
//    {
//        pthread_mutex_lock(&mutex_ka);
//
//        while (!is_ka) {
//
//            pthread_cond_wait(&cond_ka, &mutex_ka);
//        }
//
//        is_ka = false;
//
//        ka_src.copyTo(src_copy);//Tracker _src has gotten data in thread
//        Track.AS.updateData(send_data.data,send_data.quat);
//        angle_get = send_data.dat_is_get;
//        mode_temp = send_data.mode;
//        armors = send_data.armors;
//        time_temp = send_data.tim;
//        pthread_mutex_unlock(&mutex_ka);
//        if(angle_get)
//        {
//            if (mode_temp == 0x21)
//            {
//                Track.AS.bullet_speed = 28.0;
//                if (Track.locateEnemy(src_copy,armors,time_temp))
//                {
//                    vdata = { Track.pitch, Track.yaw, 0x31 };
//                }
//                else
//                {
//                    //    原数据，无自瞄
//                    vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x31 };
//                }
//                Track.show();
//                port.TransformData(vdata);
//                port.send();
//            }
//        }
//    }
//}
