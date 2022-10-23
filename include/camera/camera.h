#pragma once
#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h> //ipiimage
class Camera
{
public:
    Camera() = default;

    bool init();
    bool read_frame_rgb();
    bool release_data();
    ~Camera();


    CameraSdkStatus status; //函数返回的错误类型。

    // 调用CameraEnumerateDevice前，先设置CameraNums = 16，表示最多只读取16个设备。
    // 如果需要枚举更多的设备，请更改CameraList数组的大小和CameraNums的值。
    tSdkCameraDevInfo CameraList[1];

    int CameraNums = 1;

    CameraHandle hCamera; //相机的句柄。

    tSdkCameraCapbility CameraInfo; //相机的特性。

    BYTE* pFrameBuffer = NULL; //分配 RGB buffer。

    // 获得一帧图像数据。
    tSdkFrameHead FrameHead;
    BYTE* pRawData;

    IplImage* iplImage = nullptr;    //储存在缓存区地原始图像数据。
};
