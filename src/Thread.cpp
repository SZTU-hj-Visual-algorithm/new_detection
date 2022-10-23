#include "Thread.h"
#include <cstdio>
#include <opencv2/opencv.hpp>

using namespace cv;

bool is_continue = true;

typedef struct form
{
    vector<Armor> Enemy_s;
    float a[4];
    int is_get;
    int mode;
    int da_is_get;
}form;

form send_data;

Mat ka_src_get;

SerialPort port("/dev/ttyUSB");

void* Build_Src(void* PARAM)
{
    Mat get_src;
    auto camera_warper = new Camera;
    printf("camera_open\n");
    if (camera_warper->init())
    {
        printf("1\n");
        while (is_continue && !(waitKey(10) == 27))
        {
            if (camera_warper->read_frame_rgb())
            {
                get_src = cv::cvarrToMat(camera_warper->iplImage).clone();
                pthread_mutex_lock(&mutex_new);
                {
                    get_src.copyTo(src);
                    is_start = true;
                    pthread_cond_signal(&cond_new);
                    pthread_mutex_unlock(&mutex_new);

                    camera_warper->release_data();
                }
            }
            else
            {
                src = cv::Mat();
            }

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
    ArmorDetector autoShoot;
    vector<Armor> autoTarget;

    Mat src_copy;

    port.initSerialPort();

    sleep(2);    // 会影响程序运行时间吗
    printf("Armor_open\n");
    while (is_continue)
    {
        pthread_mutex_lock(&mutex_new);

        while (!is_start) {
            pthread_cond_wait(&cond_new, &mutex_new);
        }

        is_start = false;

        src.copyTo(src_copy);

        pthread_mutex_unlock(&mutex_new);
        float lin[4];
        int mode_temp/* = 0x22*/;
        //lin[0] = 0.0;
        //lin[1] = 5.0;
        //lin[2] = 5.0;
        //lin[3] = 25.0;
        bool small_energy = false;
        int lin_is_get;
        //lin_is_get = true;
        lin_is_get = port.get_Mode1(mode_temp, lin[0], lin[1], lin[2], lin[3],autoShoot.enermy_color);
        //mode_temp = 0x22;
        autoShoot.enermy_color = RED;
        //printf("mode:%x\n",shibie.enermy_color);
        //printf("mode_temp:%x\n",mode_temp);
        //printf("speed:%lf\n",lin[3]);
        if (mode_temp == 0x21)
        {
            vector<Armor> aim_get = autoShoot.autoAim(src);

            pthread_mutex_lock(&mutex_ka);
            send_data.Enemy_s = aim_get;
            send_data.a[0] = lin[0];
            send_data.a[1] = lin[1];
            send_data.a[2] = lin[2];
            send_data.a[3] = lin[3];
            src_copy.copyTo(ka_src_get);
            send_data.mode = mode_temp;
            send_data.is_get = lin_is_get;
            is_ka = true;
            pthread_cond_signal(&cond_ka);
            pthread_mutex_unlock(&mutex_ka);
        }


    }
}

/*

void* Kal_predict(void* PARAM)
{
    ArmorTracker autoTrack;
    headAngle sendAngle;

    int mode_temp;

    form get_data;
    sleep(3);
    printf("kal_open\n");
    int is_get;
    int mode;
    int is_send;
    float ji_pitch,ji_yaw;
    int pan_wu = 0;
    vector<Armor> Aims;

    while (is_continue)
    {
        pthread_mutex_lock(&mutex_ka);

        while (!is_ka) {

            pthread_cond_wait(&cond_ka, &mutex_ka);
        }

        is_ka = false;


        is_get=send_data.is_get;
        mode = send_data.mode;
        is_send = send_data.da_is_get;

        pthread_mutex_unlock(&mutex_ka);
        Aims = send_data.Enemy_s;
        if(is_get)
        {
            if (mode == 0x21)
            {
                sendAngle = autoTrack.finalResult(Aims, start);

                if (ka.predict(mubiao, kf, time_count))
                {
                    time_count = (double)getTickCount();
                    ji_pitch=ka.send.pitch;
                    ji_yaw = ka.send.yaw;
                    vdata = { -ji_pitch, -ji_yaw, 0x31 };
                    printf("yaw:%f\npitch:%f\n", -ka.send.yaw, -ka.send.pitch);
                    port.TransformData(vdata);
                    port.send();
                    pan_wu = 0;
                }
                else
                {
                    if(pan_wu<=12)
                    {

                        vdata = { -ji_pitch, -ji_yaw, 0x31 };
                        printf("yaw:%f\npitch:%f\npan_wu:%d\n",-ji_yaw, -ji_pitch,pan_wu);
                        port.TransformData(vdata);
                        port.send();
                        pan_wu++;
                    }else
                    {
                        ji_yaw = 0.0 - ka.ab_yaw;
                        ji_pitch = 0.0 - ka.ab_pitch;
                        ka.sp_reset(kf);
                        vdata = { -ji_pitch, -ji_yaw, 0x32 };
                        //printf("real none!!");
                        //printf("chong\n");
                        port.TransformData(vdata);
                        port.send();
                    }
                }
            }
        }
    }
}


*/