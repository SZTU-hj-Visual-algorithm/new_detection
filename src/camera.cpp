#include "camera.h"
#include <iostream>



bool Camera::init()
{
	//初始化sdk相机
	CameraSdkInit(1);
	//获取连接到当前pc端的相机设备
	int camera_status = CameraEnumerateDevice(camera_list, &pid);
	if (camera_status != CAMERA_STATUS_SUCCESS)
	{
		std::cout << "CameraEnumerateDevice fail with" << camera_status << "!" << std::endl;
	}

	//初始化相机设备并创建相机句柄
	if (CameraInit(camera_list, -1, -1, &h_camera) != CAMERA_STATUS_SUCCESS)
	{
		CameraUnInit(h_camera);
	}
	
	//获取相机设备的特性结构体
	auto status = CameraGetCapability(h_camera, &capbility);
	
	if (status != CAMERA_STATUS_SUCCESS)
	{
		std:: cout << "get capbility failed" << std::endl;
	}

	//创建rgb图像数据缓冲区
	rgb_buffer = (unsigned char*)malloc(capbility.sResolutionRange.iHeightMax *
		capbility.sResolutionRange.iWidthMax * 3
	);


//    CameraSetContrast(h_camera,200);
//    CameraSetSaturation(h_camera,1200);
//    CameraSetSharpness(h_camera,10)；
    CameraSetExposureTime(h_camera,1400);
    CameraSetAnalogGain(h_camera,150);

	//相机开始图像采集
	CameraPlay(h_camera);

	//设置getimagebuffer的输出格式
	if (capbility.sIspCapacity.bMonoSensor)
	{
		channel = 1;
		CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_MONO8);

	}
	else
	{
		channel = 3;
		CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);

	}
	return true;
}

bool Camera::read_frame_raw(cv::Mat &src)
{
	//获取相机句柄对应地相机设备采获的图像头指针和将图像数据存储到缓存区中
	if (CameraGetImageBuffer(h_camera, &frame_h, &pbybuffer, 1000) == CAMERA_STATUS_SUCCESS)
	{
		if (ipiimage)
		{
			//将图像数据头指针释放
			cvReleaseImageHeader(&ipiimage);
		}
		//创建raw图像数据的头指针
		ipiimage = cvCreateImageHeader(cvSize(frame_h.iWidth, frame_h.iHeight), IPL_DEPTH_8U, 1);


		CameraFlipFrameBuffer(rgb_buffer, &frame_h, 3);

		//将原始图像数据缓冲区中的图像数据赋给头指针
		cvSetData(ipiimage, pbybuffer, frame_h.iWidth);

		//将IPIImage原始数据图像转化为opencv的Mat图像类型
		src = cv::cvarrToMat(ipiimage);
		//将图像缓冲区中的图像数据释放掉
		CameraReleaseImageBuffer(h_camera, pbybuffer);

		return true;

	}

	else
	{
		src = cv::Mat();
		return false;
	}
}

bool Camera::read_frame_rgb(cv::Mat& src)
{
	//从相机句柄对应的相机设备中获取图像数据头指针和将图像数据保存在数据缓冲区中
	if (CameraGetImageBuffer(h_camera, &frame_h, &pbybuffer, 1000) == CAMERA_STATUS_SUCCESS)
	{
		CameraImageProcess(h_camera, pbybuffer, rgb_buffer, &frame_h);
		if (ipiimage)
		{
			//释放掉图像数据的头指针，相当于释放全部数据
			cvReleaseImageHeader(&ipiimage);
		}

		//新建一个图像数据头指针
		ipiimage = cvCreateImageHeader(cvSize(frame_h.iWidth, frame_h.iHeight), IPL_DEPTH_8U, channel);

		CameraFlipFrameBuffer(rgb_buffer, &frame_h, 4);

		//将rgb图像数据缓冲区里的图像数据给头指针指向
		cvSetData(ipiimage, rgb_buffer, frame_h.iWidth * channel);

		//将IPIImage格式的图像数据转为opencv的Mat图像类型
		src = cv::cvarrToMat(ipiimage);

		//释放掉图像数据缓冲区的数据，防止引起数据阻塞
		CameraReleaseImageBuffer(h_camera, pbybuffer);

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
	CameraUnInit(h_camera);
}