#include "camera.h"
#include <cstring>
#include <iostream>

void Camera::re()
{
    // 一个进程调用一次，初始化 SDK 为中文接口
    CameraSdkInit(1);

    // 枚举设备，获得设备列表
    int CameraNums = 16;
    status = CameraEnumerateDevice(CameraList, &CameraNums);


    // 只初始化第一个相机。
    // (-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数。
    status = CameraInit(&CameraList[0], -1, -1, &hCamera);


    // 获得该相机的特性描述
    CameraGetCapability(hCamera, &CameraInfo);

    // 设置相机为软触发模式，并且把一次触发的帧数固定为1
    CameraSetTriggerMode(hCamera, 0);
    //CameraSetTriggerCount(hCamera, 1);

    // 手动曝光，曝光时间 2ms
    CameraSetAeState(hCamera, FALSE);
    CameraSetExposureTime(hCamera, 2 * 1000);
    // 按 RGB 顺序提供颜色增益
    CameraSetGain(hCamera, 120, 110, 135);
    CameraSetAnalogGain(hCamera, 32);  //该值增大后会提升图像背景噪声
//    CameraSetContrast(h_camera, 200);
//    CameraSetSaturation(h_camera, 1200);
//    CameraSetSharpness(h_camera, 10);

    // 让SDK内部取图线程开始工作
    CameraPlay(hCamera);

    // 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
    UINT FrameBufferSize = CameraInfo.sResolutionRange.iWidthMax * CameraInfo.sResolutionRange.iHeightMax * 3;

    // 分配 RGB buffer，用来存放ISP输出的图像
    // 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
    pFrameBuffer = (BYTE *)CameraAlignMalloc(FrameBufferSize, 16);

    // 由于相机目前处于软触发模式，需要软件发送指令通知相机拍照（为了避免意外取到相机缓存中的旧图片，在给触发指令前先清空了缓存）
    //CameraClearBuffer(hCamera);
    //CameraSoftTrigger(hCamera);

    auto camera_warrper = new Camera;
    auto time_start = std::chrono::steady_clock::now();

    std::string path = "/home/lmx/1.avi";

    CameraSdkStatus z = CameraInitRecord(hCamera, 0, "\\home\\lmx\\a.avi", FALSE, 100, 30);
    printf("%d   \n",z);

    while(true){
        // 获得一帧图像数据。
        status = CameraGetImageBuffer(hCamera, &FrameHead, &pRawData, 2000);
        if (status == CAMERA_STATUS_SUCCESS)
        {
            //将获得的相机原始输出图像数据进行处理，迭加饱和度、颜色增益和校正、降噪等处理效果，最后得到 RGB24 格式的图像数据。
            CameraImageProcess(hCamera, pRawData, pFrameBuffer, &FrameHead);
            //设置为 BGR24 格式的图像数据。
            CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);



            CameraPushFrame(hCamera,pFrameBuffer,&FrameHead);

            CameraReleaseImageBuffer(hCamera, pRawData);
        }




        auto time_cap = std::chrono::steady_clock::now();
        int time_stamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count()); // 获取时间戳

        if(time_stamp > 1000){

            CameraStopRecord(hCamera);
            break;
        }

    }



    //反初始化相机。
    if (hCamera > 0)
    {
        CameraUnInit(hCamera);
        hCamera = 0;
    }

    //释放 RGB buffer。
    if (pFrameBuffer)
    {
        CameraAlignFree(pFrameBuffer);
        pFrameBuffer = NULL;
    }


}

bool Camera::init()
{

    // 一个进程调用一次，初始化 SDK 为中文接口
    CameraSdkInit(1);

    // 枚举设备，获得设备列表
    status = CameraEnumerateDevice(CameraList, &CameraNums);
    if (status != CAMERA_STATUS_SUCCESS)
    {
        printf("No camera was found!");
        return -1;
    }

    // 只初始化第一个相机。
    // (-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数。
    status = CameraInit(&CameraList[0], -1, -1, &hCamera);
    if (status != CAMERA_STATUS_SUCCESS)
    {
        printf("Failed to init the camera! Error code is %d", status);
        CameraUnInit(hCamera);
        return -1;
    }

    // 获得该相机的特性描述
    status = CameraGetCapability(hCamera, &CameraInfo);
    if (status != CAMERA_STATUS_SUCCESS)
    {
        std:: cout << "get capbility failed" << std::endl;
    }

    // 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
    UINT FrameBufferSize = CameraInfo.sResolutionRange.iWidthMax * CameraInfo.sResolutionRange.iHeightMax * 3;

    // 分配 RGB buffer，用来存放ISP输出的图像
    // 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
    pFrameBuffer = (unsigned char*)malloc(FrameBufferSize);


    // 设置相机为软触发模式，并且把一次触发的帧数固定为1
//    status = CameraSetTriggerMode(hCamera, 1);
//    if (status != CAMERA_STATUS_SUCCESS)
//    {
//        printf("Failed to 1! Error code is %d", status);
//        CameraUnInit(hCamera);
//    }
//    status = CameraSetTriggerCount(hCamera, 1);
//    if (status != CAMERA_STATUS_SUCCESS)
//    {
//        printf("Failed to 2! Error code is %d", status);
//        CameraUnInit(hCamera);
//    }

    // 手动曝光，曝光时间 2ms
    CameraSetAeState(hCamera, FALSE);
    CameraSetExposureTime(hCamera, 2 * 1000);
    // 按 RGB 顺序提供颜色增益
//    CameraSetGain(hCamera, 120, 110, 135);
    CameraSetAnalogGain(hCamera, 150);  //该值增大后会提升图像背景噪声
    //    CameraSetContrast(h_camera, 200);
    //    CameraSetSaturation(h_camera, 1200);
    //    CameraSetSharpness(h_camera, 10);

    // 让SDK内部取图线程开始工作
    CameraPlay(hCamera);

    return true;
}

bool Camera::read_frame_rgb(cv::Mat& src)
{

    // 由于相机目前处于软触发模式，需要软件发送指令通知相机拍照（为了避免意外取到相机缓存中的旧图片，在给触发指令前先清空了缓存）
//    status = CameraClearBuffer(hCamera);
//    if (status != CAMERA_STATUS_SUCCESS)
//    {
//        printf("Failed to 2! Error code is %d", status);
//        CameraUnInit(hCamera);
//    }
//    status = CameraSoftTrigger(hCamera);
//    if (status != CAMERA_STATUS_SUCCESS)
//    {
//        printf("Failed to 2! Error code is %d", status);
//        CameraUnInit(hCamera);
//    }

    // 获得一帧图像数据。
    status = CameraGetImageBuffer(hCamera, &FrameHead, &pRawData, 2000);
    if (status == CAMERA_STATUS_SUCCESS)
    {
        //将获得的相机原始输出图像数据进行处理，迭加饱和度、颜色增益和校正、降噪等处理效果，最后得到 RGB24 格式的图像数据。
        CameraImageProcess(hCamera, pRawData, pFrameBuffer, &FrameHead);

        //设置为 BGR24 格式的图像数据。
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
        //创建图像头，不分配图像数据。
        if (iplImage)
        {
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(FrameHead.iWidth, FrameHead.iHeight), IPL_DEPTH_8U, 3);

        //设置图像数据。
        cvSetData(iplImage, pFrameBuffer, FrameHead.iWidth * 3);

        //将 IplImage 转换成 Mat。
        src = cv::cvarrToMat(iplImage);

        //释放 CameraGetImageBuffer 的到的 RAW 数据缓冲区的使用权。
        CameraReleaseImageBuffer(hCamera, pRawData);

        return true;
    }
    else
    {
        src = cv::Mat();
        return false;
    }

}

Camera::~Camera()
{
    CameraUnInit(hCamera);
}