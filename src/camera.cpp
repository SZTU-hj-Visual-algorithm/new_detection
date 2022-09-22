#include "camera.h"
#include <iostream>



bool Camera::init()
{
    //��ʼ��sdk���
    CameraSdkInit(1);
    //��ȡ���ӵ���ǰpc�˵�����豸
    int camera_status = CameraEnumerateDevice(camera_list, &pid);
    if (camera_status != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "CameraEnumerateDevice fail with" << camera_status << "!" << std::endl;
    }

    //��ʼ������豸������������
    if (CameraInit(camera_list, -1, -1, &h_camera) != CAMERA_STATUS_SUCCESS)
    {
        CameraUnInit(h_camera);
    }
	
    //��ȡ����豸�����Խṹ��
    auto status = CameraGetCapability(h_camera, &capbility);
	
    if (status != CAMERA_STATUS_SUCCESS)
    {
        std:: cout << "get capbility failed" << std::endl;
    }

    //����rgbͼ�����ݻ�����
    rgb_buffer = (unsigned char*)malloc(capbility.sResolutionRange.iHeightMax *
            capbility.sResolutionRange.iWidthMax * 3
            );


    //    CameraSetContrast(h_camera,200);
    //    CameraSetSaturation(h_camera,1200);
    //    CameraSetSharpness(h_camera,10)��
    CameraSetExposureTime(h_camera,1700);
    CameraSetAnalogGain(h_camera,100);

    //�����ʼͼ��ɼ�
    CameraPlay(h_camera);

    //����getimagebuffer�������ʽ
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
    //��ȡ��������Ӧ������豸�ɻ��ͼ��ͷָ��ͽ�ͼ�����ݴ洢����������
    if (CameraGetImageBuffer(h_camera, &frame_h, &pbybuffer, 1000) == CAMERA_STATUS_SUCCESS)
    {
        if (ipiimage)
        {
            //��ͼ������ͷָ���ͷ�
            cvReleaseImageHeader(&ipiimage);
        }
        //����rawͼ�����ݵ�ͷָ��
        ipiimage = cvCreateImageHeader(cvSize(frame_h.iWidth, frame_h.iHeight), IPL_DEPTH_8U, 1);


        CameraFlipFrameBuffer(rgb_buffer, &frame_h, 3);

        //��ԭʼͼ�����ݻ������е�ͼ�����ݸ���ͷָ��
        cvSetData(ipiimage, pbybuffer, frame_h.iWidth);

        //��IPIImageԭʼ����ͼ��ת��Ϊopencv��Matͼ������
        src = cv::cvarrToMat(ipiimage);
        //��ͼ�񻺳����е�ͼ�������ͷŵ�
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
    //����������Ӧ������豸�л�ȡͼ������ͷָ��ͽ�ͼ�����ݱ��������ݻ�������
    if (CameraGetImageBuffer(h_camera, &frame_h, &pbybuffer, 1000) == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(h_camera, pbybuffer, rgb_buffer, &frame_h);
        if (ipiimage)
        {
            //�ͷŵ�ͼ�����ݵ�ͷָ�룬�൱���ͷ�ȫ������
            cvReleaseImageHeader(&ipiimage);
        }

        //�½�һ��ͼ������ͷָ��
        ipiimage = cvCreateImageHeader(cvSize(frame_h.iWidth, frame_h.iHeight), IPL_DEPTH_8U, channel);

        CameraFlipFrameBuffer(rgb_buffer, &frame_h, 4);

        //��rgbͼ�����ݻ��������ͼ�����ݸ�ͷָ��ָ��
        cvSetData(ipiimage, rgb_buffer, frame_h.iWidth * channel);

        //��IPIImage��ʽ��ͼ������תΪopencv��Matͼ������
        src = cv::cvarrToMat(ipiimage);

        //�ͷŵ�ͼ�����ݻ����������ݣ���ֹ������������
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