#pragma once
#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
class Camera
{
public:
	Camera() = default;
	
	bool init();
	bool read_frame_rgb(cv::Mat& src);
	bool read_frame_raw(cv::Mat& src);
//	bool transform_src_data(cv::Mat &src);
	bool release_data();
	bool camera_record();
	bool record_start();
	
	~Camera();
	
	
	int record_state = RECORD_STOP;//用来记录录像状态
	IplImage* ipiimage = nullptr;    //储存在缓存区地原始图像数据

private:
	
	tSdkCameraDevInfo camera_list[1];
	INT pid = 1;
	CameraHandle h_camera;
	tSdkCameraCapbility capbility;
	tSdkFrameHead frame_h;//图像头指针
	tSdkFrameHead record_hframe;//录像所使用的图像头指针
	unsigned char* rgb_buffer = nullptr;//rgb图像数据缓冲区
	int channel = 3 ;
	BYTE* pbybuffer = nullptr;//原始图像数据缓冲区
	char path[100] = "1.avi";
	
};
