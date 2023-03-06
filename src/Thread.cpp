#include "Thread.h"
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <chrono>


using namespace cv;
//using namespace robot_detection;
using namespace std;

SerialPort port("/dev/ttyUSB0");


//one2two
form _send_data;
Mat src;

//two2three
form send_data;
Mat ka_src;




void* Build_Src(void* PARAM)
{
    int lin_is_get/* = true*/;
    int mode_temp;
	float lin[3];
	float quat[4];
	chrono_time time_temp;
	Mat get_src;
    auto camera_warper = new Camera;
	port.initSerialPort();
	printf("camera_open-\n");
    // chrono
//    auto start = chrono::high_resolution_clock::now();
	if (camera_warper->init())
	{
		printf("1-real\n");
		while (is_continue && (waitKey(1) != 27))
		{
            // chrono
//            auto end = chrono::high_resolution_clock::now();    //结束时间
			if (camera_warper->read_frame_rgb())
			{
				get_src = camera_warper->src.clone();

				lin_is_get = port.get_Mode1_new(mode_temp, lin[0], lin[1], lin[2],quat);
				time_temp = chrono::high_resolution_clock::now();
                pthread_mutex_lock(&mutex_new);
                {
                    get_src.copyTo(src);
					_send_data = {mode_temp,
								 lin_is_get,
								 {lin[0],lin[1],lin[2]},
								 {quat[0],quat[1],quat[2],quat[3]},
								 vector<Armor>(),
								 time_temp};
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
	int color_get;

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
        color_get = _send_data.dat_is_get;
        time_temp = _send_data.tim;
		//imshow("src_copy",src_copy);

		pthread_mutex_unlock(&mutex_new);
        if(color_get)
        {
            if (mode_temp == 0x21)
            {
                Targets = Detect.autoAim(src_copy);
                if(!Targets.empty())std::cout<<"------------Get Targets--------------"<<std::endl;
                else std::cout<<"------------No Targets--------------"<<std::endl;
                pthread_mutex_lock(&mutex_ka);
                send_data = {mode_temp,
							 color_get,
							 {Detect.ab_pitch,Detect.ab_yaw,Detect.bullet_speed},
							 {Detect.quaternion[0],Detect.quaternion[1],Detect.quaternion[2],Detect.quaternion[3]},
							 Targets,
							 time_temp};
//                send_data.armors = Targets;
//                send_data.data[0] = Detect.ab_pitch;
//                send_data.data[1] = Detect.ab_yaw;
//                send_data.data[2] = Detect.bullet_speed;
//                send_data.mode = mode_temp;
//                send_data.dat_is_get = color_get;
//                send_data.tim = time_temp;
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
	VisionData vdata;
	vector<Armor> armors;
    ArmorTracker Track;
	int mode_temp;
	int angle_get;
    chrono_time time_temp;
    Mat src_copy;

    sleep(3);
    printf("kal_open\n");
	while (is_continue)
	{
		pthread_mutex_lock(&mutex_ka);

		while (!is_ka) {

			pthread_cond_wait(&cond_ka, &mutex_ka);
		}

		is_ka = false;

		ka_src.copyTo(src_copy);//Tracker _src has gotten data in thread
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
					printf("--------------Thread End----------\n");
				}
				else
				{
                     //    原数据，无自瞄
						vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x31 };
						port.TransformData(vdata);
						port.send();
					printf("--------------Thread End----------\n");
				}
                Track.show();
              	port.TransformData(vdata);
				port.send();
			}
		}
	}
}


