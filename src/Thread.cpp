#include "thread.h"
#include<fstream>
using namespace cv;
using namespace std;

//#define UART_PORT
#define USB_PORT
//#define SAVE_FIRST
//#define SAVE_SECOND

#ifdef UART_PORT
SerialPort port;    // uart
#endif // UART_PORT

#ifdef USB_PORT
SerialMain serial;  // usb
#endif // USB_PORT


int t = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
string ss = to_string(t);

#ifdef SAVE_FIRST
// 一线程录制视频
string outputPath_1 = "../video/"  +ss+ string( "_1.avi");
VideoWriter outputVideo_1(outputPath_1, VideoWriter::fourcc('M','P','4','2'), 25, Size(1280, 1024));
// 一线程串口数据保存
string serialPath_1 = "../video/" + ss + string("_1.txt");
ofstream outfile_1(serialPath_1);
#endif // SAVE_FIRST

#ifdef SAVE_SECOND
// 二线程录制视频
string outputPath_2 = "../video/"  +ss+ string( "_2.avi");
VideoWriter outputVideo_2(outputPath_2, VideoWriter::fourcc('M','P','4','2'), 25, Size(1280, 1024));
// 二线程串口数据保存
string serialPath_2 = "../video/" + ss + string("_2.txt");
ofstream outfile_2(serialPath_2);
#endif // SAVE_SECOND

//one_2_two
form _send_data;
Mat src;

void* Sample(void* PARAM)
{
#ifdef USB_PORT
    int first_get/* = true*/;   // usb
#endif // USB_PORT

    chrono_time time_temp;
    Mat get_src;
    auto camera_warper = new Camera;
    fmt::print("camera_open-\n");

    if (camera_warper->init())
    {
        while (is_continue && (waitKey(1) != 27))
        {
            if (camera_warper->read_frame_rgb())
            {
                get_src = camera_warper->src.clone();

#ifdef UART_PORT
//				lin_is_get = port.get_Mode1_new(mode_temp, lin[0], lin[1], lin[2], quat);    // uart
#endif // UART_PORT

#ifdef USB_PORT
                first_get = serial.ReceiverMain();                                          // usb
#endif // USB_PORT

                time_temp = chrono::high_resolution_clock::now();
                pthread_mutex_lock(&mutex_new);
                {
                    get_src.copyTo(src);

#ifdef USB_PORT
                    // usb  //TODO: 这个值需要变化，用来模式切换serial.vision_msg_.mode
                    // TODO: 这个以后用来决定自喵，基地模式只识别基地，什么情况识别什么，可自定义程度极高
                    _send_data = {0x21,first_get, //! 0x21应该换成serial.vision_msg_.mode赋值，方便调试默认给的0x21
                                  {serial.vision_msg_.pitch,
                                   serial.vision_msg_.yaw,
                                   serial.vision_msg_.shoot},
                                  {serial.vision_msg_.quaternion[0],
                                   serial.vision_msg_.quaternion[1],
                                   serial.vision_msg_.quaternion[2],
                                   serial.vision_msg_.quaternion[3]},
                                  vector<Armor>(),time_temp};
#endif // USB_PORT

                    is_start = true;
                    pthread_cond_signal(&cond_new);
                    pthread_mutex_unlock(&mutex_new);
                    imshow("src",src);

#ifdef SAVE_FIRST
                    outputVideo_1 << src;

                    time_t now_c = chrono::system_clock::to_time_t(_send_data.tim);
                    outfile_1 << _send_data.mode <<" ";
                    outfile_1 << _send_data.dat_is_get<<" ";
                    for(int i=0;i<3;i++)
                    {
                        outfile_1<<_send_data.data[i]<<" ";
                    }
                    for(int i=0;i<4;i++)
                    {
                        outfile_1<<_send_data.quat[i]<<" ";
                    }
                    outfile_1<<put_time(localtime(&now_c),"%F %T")<<"\n";
#endif // SAVE_FIRST

                    camera_warper->release_data();
                }
            }
            else
            {
                src = cv::Mat();
            }
        }

#ifdef SAVE_FIRST
        outputVideo_1.release();
        outfile_1.close();
#endif // SAVE_FIRST

        camera_warper->~Camera();
        pthread_mutex_unlock(&mutex_new);
        is_continue = false;
    }
    else
    {
        fmt::print("No camera!!\n");
        is_continue = false;
    }
}

void* Implement(void* PARAM)
{
    ArmorDetector Detect;
    std::vector<Armor> Targets;
    Mat src_copy;
    chrono_time time_temp;
    int mode_temp;
    int second_get;
    vector<double> vdata(3);
    ArmorTracker Track;

    //!< 感觉可以不需要sleep，可以用一个Peterson's Algorithm来进行临界区互斥同步（猜想）
    sleep(2);
    fmt::print("Armor_open\n");
    while (is_continue)
    {
        pthread_mutex_lock(&mutex_new);

        while (!is_start) {
            pthread_cond_wait(&cond_new, &mutex_new);
        }
        is_start = false;

        src.copyTo(src_copy);
        Detect.updateData(_send_data.data,_send_data.quat);////
        Track.AS.updateData(_send_data.data, _send_data.quat);////每次都要重设
        mode_temp = _send_data.mode;
        second_get = _send_data.dat_is_get;
        time_temp = _send_data.tim;
        pthread_mutex_unlock(&mutex_new);
        if(second_get && mode_temp == 0x21)
        {
//            printf("[mode_temp]:    |%x\n",mode_temp);
            Targets = Detect.autoAim(src_copy);

            Track.AS.bullet_speed = 28.0;

            if (Track.locateEnemy(src_copy,Targets,time_temp))
            {
                cv::circle(Track._src,cv::Point(640,20),20,cv::Scalar(0,255,0),-1);
                vdata = { Track.pitch, Track.yaw, 0x31 };
            }
            else
            {
                cv::circle(Track._src,cv::Point(640,20),20,cv::Scalar(0,0,255),-1);
                vdata = { Track.AS.ab_pitch, Track.AS.ab_yaw, 0x32 };
            }
            cv::putText(Track._src,std::to_string(Targets.size()) +" ARMOR",cv::Point2f(1280 - 200,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
            Track.show();

#ifdef SAVE_SECOND
            outputVideo_2 << Track._src;

            time_t now_c = chrono::system_clock::to_time_t(_send_data.tim);
            outfile_2 << _send_data.mode <<" ";
            outfile_2 << _send_data.dat_is_get<<" ";
            for(int i=0;i<3;i++)
            {
                outfile_2<<_send_data.data[i]<<" ";
            }
            for(int i=0;i<4;i++)
            {
                outfile_2<<_send_data.quat[i]<<" ";
            }
            outfile_2<<put_time(localtime(&now_c),"%F %T")<<"\n";
#endif // SAVE_SECOND

#ifdef USB_PORT
            serial.SenderMain(vdata);
#endif // USB_PORT
        }
    }

#ifdef SAVE_SECOND
    outputVideo_2.release();
    outfile_2.close();
#endif // SAVE_SECOND
}