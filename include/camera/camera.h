#pragma once
#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include "opencv2/core/core_c.h"
//using namespace cv;

class Camera
{
public:
    Camera() = default;

    bool init();
    bool read_frame_rgb(cv::Mat &src);
    bool read_frame_raw(cv::Mat& src);

    ~Camera();
private:
    IplImage* ipiimage = nullptr;    //�����ڻ�������ԭʼͼ������
    tSdkCameraDevInfo camera_list[1];
    INT pid = 1;
    CameraHandle h_camera;
    tSdkCameraCapbility capbility;
    tSdkFrameHead frame_h;//ͼ��ͷָ��
    unsigned char* rgb_buffer = nullptr;//rgbͼ�����ݻ�����
    int channel;
    bool init_done;
    BYTE* pbybuffer = nullptr;//ԭʼͼ�����ݻ�����
    tSdkImageResolution *pImageResolution;

};