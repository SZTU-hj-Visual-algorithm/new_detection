#pragma once
#ifndef _MVCAMAPI_H_
#define _MVCAMAPI_H_


#ifdef DLL_EXPORT
#define MVSDK_API extern "C" __declspec(dllexport)
#else
#define MVSDK_API extern "C" __declspec(dllimport)
#endif

#include <windows.h>
//#include "camera_api.h"
#include <camera_status.h>
#include <camera_define.h>

/// @ingroup API_BASIC
/// \~chinese
/// \brief ��ʼ��SDK���ԡ��ú������������������ڼ�ֻ��Ҫ����һ�Ρ�
/// \param [in] iLanguageSel ����ѡ��SDK�ڲ���ʾ��Ϣ�ͽ��������,0:��ʾӢ��,1:��ʾ���ġ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Initialize the SDK language. This function only needs to be called once during the entire process run.
/// \param [in] iLanguageSel The language used to select the prompt information and interface of the SDK. 0: English, 1: Chinese.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSdkInit(
	int     iLanguageSel
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ����ϵͳѡ�ͨ����Ҫ��CameraInit�����֮ǰ���úã�
/// \param [in] optionName ѡ��("NumBuffers", "3")
/// \param [in] value ֵ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Configure system options (usually required before CameraInit turns on the camera)
/// \param [in] optionName option name("NumBuffers", "3")
/// \param [in] value setting value
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetSysOption(
	char const* optionName,
	char const* value
);

/// @ingroup API_ENUM
/// \~chinese
/// \brief ö���豸���������豸�б�
/// \param [out] pCameraList �豸�б�����ָ��
/// \param [inout] piNums �豸�ĸ���ָ�룬����ʱ����pCameraList�����Ԫ�ظ�������������ʱ������ʵ���ҵ����豸����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ���
/// \warning piNumsָ���ֵ�����ʼ�����Ҳ�����pCameraList����Ԫ�ظ����������п�������ڴ����
/// \note ���ص������Ϣ�б������acFriendlyName����ġ�������Խ���������ֱ��Ϊ��Camera1���͡�Camera2�������ֺ�����Ϊ��Camera1�����������ǰ�棬��Ϊ��Camera2��������ź��档
/// \~english
/// \brief Enumerate devices and establish a list of devices
/// \param [out] pCameraList Device list array pointer
/// \param [inout] piNums The number of pointers to the device, the number of elements passed to the pCameraList array at the time of the call. When the function returns, the number of devices actually found is saved.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \warning piNums The value pointed to must be initialized and does not exceed the number of pCameraList array elements, otherwise it may cause memory overflow
/// \note The list of returned camera information will be sorted according to acFriendlyName. For example, after changing the two cameras to the names of "Camera1" and "Camera2," the camera named "Camera1" will be in front, and the camera named "Camera2" will be behind the row.
MVSDK_API CameraSdkStatus __stdcall CameraEnumerateDevice(
	tSdkCameraDevInfo* pCameraList,
	INT* piNums
);

/// @ingroup API_ENUM
/// \~chinese
/// \brief ö���豸���������豸�б��ڵ���@link #CameraInitEx @endlink֮ǰ��������øú���ö���豸��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Enumerate devices and create a list of devices. Before calling @link #CameraInitEx @endlink, this function must be called to enumerate the device.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API INT __stdcall CameraEnumerateDeviceEx(
);

/// @ingroup API_OPEN
/// \~chinese
/// \brief ����豸�Ƿ��Ѿ�����
/// \param [in] pCameraInfo �豸��ö����Ϣ�ṹ��ָ�룬��@link #CameraEnumerateDevice @endlink��á�
/// \param [out] pOpened �豸��״ָ̬�룬�����豸�Ƿ񱻴򿪵�״̬��TRUEΪ�򿪣�FALSEΪ���С�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Check if the device has been opened
/// \param [in] pCameraInfo Device enumeration information structure pointer, obtained by @link #CameraEnumerateDevice @endlink
/// \param [out] pOpened The device's status pointer returns whether the device is turned on. TRUE is on and FALSE is idle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraIsOpened(
	tSdkCameraDevInfo* pCameraInfo,
	BOOL* pOpened
);

/// @ingroup API_OPEN
/// \~chinese
/// \brief �����ʼ������ʼ���ɹ��󣬲��ܵ������������صĲ����ӿڡ�
/// \param [in] pCameraInfo �豸��ö����Ϣ�ṹ��ָ�룬��@link #CameraEnumerateDevice @endlink��á�
/// \param [in] emParamLoadMode �����ʼ��ʱʹ�õĲ������ط�ʽ��-1��ʾʹ���ϴ��˳�ʱ�Ĳ������ط�ʽ������ȡֵ�ο�@link #emSdkParameterMode @endlink���塣
/// \param [in] emTeam ��ʼ��ʱʹ�õĲ����顣-1��ʾ�����ϴ��˳�ʱ�Ĳ����顣
/// \param [out] pCameraHandle ����ľ��ָ�룬��ʼ���ɹ��󣬸�ָ�뷵�ظ��������Ч�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The camera is initialized. After successful initialization, other camera-related operation interfaces can be called.
/// \param [in] pCameraInfo Device enumeration information structure pointer, obtained by @link #CameraEnumerateDevice @endlink.
/// \param [in] emParamLoadMode The parameter loading method used when the camera is initialized. -1 means to use the parameter loading method from the last exit. Other values are defined in @link #emSdkParameterMode @endlink.
/// \param [in] emTeam Parameter group used during initialization. -1 means to load the parameter group from the last exit.
/// \param [out] pCameraHandle The handle pointer of the camera, after successful initialization, returns the camera's effective handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraInit(
	tSdkCameraDevInfo* pCameraInfo,
	int                 emParamLoadMode,
	int                 emTeam,
	CameraHandle* pCameraHandle
);

/// @ingroup API_OPEN
/// \~chinese
/// \brief �����ʼ������ʼ���ɹ��󣬲��ܵ������������صĲ����ӿڡ�
/// \param [in] iDeviceIndex ����������ţ�@link #CameraEnumerateDeviceEx @endlink�������������
/// \param [in] emParamLoadMode �����ʼ��ʱʹ�õĲ������ط�ʽ��-1��ʾʹ���ϴ��˳�ʱ�Ĳ������ط�ʽ������ȡֵ�ο�@link #emSdkParameterMode @endlink���塣
/// \param [in] emTeam ��ʼ��ʱʹ�õĲ����顣-1��ʾ�����ϴ��˳�ʱ�Ĳ����顣
/// \param [out] pCameraHandle ����ľ��ָ�룬��ʼ���ɹ��󣬸�ָ�뷵�ظ��������Ч�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The camera is initialized. After successful initialization, other camera-related operation interfaces can be called.
/// \param [in] iDeviceIndex The camera's index number, @link #CameraEnumerateDeviceEx @endlink returns the number of cameras.
/// \param [in] emParamLoadMode The parameter loading method used when the camera is initialized. -1 means to use the parameter loading method from the last exit. Other values are defined in @link #emSdkParameterMode @endlink.
/// \param [in] emTeam Parameter group used during initialization. -1 means to load the parameter group from the last exit.
/// \param [out] pCameraHandle The handle pointer of the camera, after successful initialization, returns the camera's effective handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraInitEx(
	int             iDeviceIndex,
	int             emParamLoadMode,
	int             emTeam,
	CameraHandle* pCameraHandle
);

/// @ingroup API_OPEN
/// \~chinese
/// \brief �����ʼ������ʼ���ɹ��󣬲��ܵ������������صĲ����ӿڡ�
/// \param [in] CameraName ����ǳơ�@link #tSdkCameraDevInfo.acFriendlyName @endlink
/// \param [out] pCameraHandle ����ľ��ָ�룬��ʼ���ɹ��󣬸�ָ�뷵�ظ��������Ч�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The camera is initialized. After successful initialization, other camera-related operation interfaces can be called.
/// \param [in] CameraName Camera friendly name.@link #tSdkCameraDevInfo.acFriendlyName @endlink
/// \param [out] pCameraHandle The handle pointer of the camera, after successful initialization, returns the camera's effective handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraInitEx2(
	char* CameraName,
	CameraHandle* pCameraHandle
);

/// @ingroup API_GRAB_CB
/// \~chinese
/// \brief ����ͼ�񲶻�Ļص��������������µ�ͼ������֡ʱ��pCallBack��ָ��Ļص������ͻᱻ���á�
/// \param [in] hCamera ����ľ����
/// \param [in] pCallBack �ص�����ָ�롣
/// \param [in] pContext �ص������ĸ��Ӳ������ڻص�����������ʱ�ø��Ӳ����ᱻ���룬����ΪNULL�������ڶ�����ʱЯ��������Ϣ��
/// \param [out] pCallbackOld ���ڷ���֮ǰ���õĻص�����������ΪNULL��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the image capture's callback function. When a new frame of image data is captured, the callback function pointed to by pCallBack is called.
/// \param [in] hCamera Camera handle.
/// \param [in] pCallBack Callback function pointer.
/// \param [in] pContext Additional parameters of the callback function, which will be passed in when the callback function is called, can be NULL. Use additional information when used with multiple cameras.
/// \param [out] pCallbackOld Returns the previously set callback function. Can be NULL.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetCallbackFunction(
	CameraHandle        hCamera,
	CAMERA_SNAP_PROC    pCallBack,
	PVOID               pContext,
	CAMERA_SNAP_PROC* pCallbackOld
);

/// @ingroup API_CLOSE
/// \~chinese
/// \brief �������ʼ�����ͷ���Դ��
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The camera is deinitialized. Release resources.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraUnInit(
	CameraHandle hCamera
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ��������������Ϣ
/// \param [in] hCamera ����ľ����
/// \param [out] pbuffer ָ�����������Ϣָ���ָ�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get camera description information
/// \param [in] hCamera Camera handle.
/// \param [out] pbuffer Pointer to the camera description information pointer.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInformation(
	CameraHandle    hCamera,
	char** pbuffer
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ����õ����ԭʼ���ͼ�����ݽ��д������ӱ��Ͷȡ���ɫ�����У��������ȴ���Ч�������õ�RGB888��ʽ��ͼ�����ݡ�
/// \param [in] hCamera ����ľ����
/// \param [in] pbyIn ����ͼ�����ݵĻ�������ַ������ΪNULL�� 
/// \param [out] pbyOut �����ͼ������Ļ�������ַ������ΪNULL��
/// \param [inout] pFrInfo ����ͼ���֡ͷ��Ϣ��������ɺ�֡ͷ��Ϣ�е�ͼ���ʽuiMediaType����֮�ı䡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The obtained raw camera output image data is processed to superimpose processing effects such as saturation, color gain and correction, noise reduction, etc. Finally, image data of RGB888 format is obtained.
/// \param [in] hCamera Camera handle.
/// \param [in] pbyIn The buffer address of the input image data cannot be NULL.
/// \param [out] pbyOut The buffer address of the image output after processing cannot be NULL.
/// \param [inout] pFrInfo After inputting the frame header information of the image, the image format uiMediaType in the frame header information will be changed after the processing is completed.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraImageProcess(
	CameraHandle        hCamera,
	BYTE* pbyIn,
	BYTE* pbyOut,
	tSdkFrameHead* pFrInfo
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ����õ����ԭʼ���ͼ�����ݽ��д������ӱ��Ͷȡ���ɫ�����У��������ȴ���Ч�������õ�RGB888��ʽ��ͼ�����ݡ�
/// \param [in] hCamera ����ľ����
/// \param [in] pbyIn ����ͼ�����ݵĻ�������ַ������ΪNULL�� 
/// \param [out] pbyOut �����ͼ������Ļ�������ַ������ΪNULL��
/// \param [inout] pFrInfo ����ͼ���֡ͷ��Ϣ��������ɺ�֡ͷ��Ϣ�е�ͼ���ʽuiMediaType����֮�ı䡣
/// \param [in] uOutFormat �������ͼ��������ʽ��������CAMERA_MEDIA_TYPE_MONO8��CAMERA_MEDIA_TYPE_RGB��CAMERA_MEDIA_TYPE_RGBA8������һ�֡�
/// \param [in] uReserved Ԥ����������������Ϊ0��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The obtained raw camera output image data is processed to superimpose processing effects such as saturation, color gain and correction, noise reduction, etc. Finally, image data of RGB888 format is obtained.
/// \param [in] hCamera Camera handle.
/// \param [in] pbyIn The buffer address of the input image data cannot be NULL.
/// \param [out] pbyOut The buffer address of the image output after processing cannot be NULL.
/// \param [inout] pFrInfo After inputting the frame header information of the image, the image format uiMediaType in the frame header information will be changed after the processing is completed.
/// \param [in] uOutFormat The output format of the image after processing. It may be one of CAMERA_MEDIA_TYPE_MONO8,CAMERA_MEDIA_TYPE_RGB,CAMERA_MEDIA_TYPE_RGBA8.
/// \param [in] uReserved Reservation parameters must be set to 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraImageProcessEx(
	CameraHandle        hCamera,
	BYTE* pbyIn,
	BYTE* pbyOut,
	tSdkFrameHead* pFrInfo,
	UINT                uOutFormat,
	UINT                uReserved
);

/// @ingroup API_DISPLAY
/// \~chinese
/// \brief ��ʼ��SDK�ڲ�����ʾģ�顣�ڵ���@link #CameraDisplayRGB24 @endlinkǰ�����ȵ��øú�����ʼ����������ڶ��ο����У�ʹ���Լ��ķ�ʽ����ͼ����ʾ(������CameraDisplayRGB24)����Ҫ���ñ������� 
/// \param [in] hCamera ����ľ����
/// \param [in] hWndDisplay ��ʾ���ڵľ����һ��Ϊ���ڵ�m_hWnd��Ա��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Initialize the display module inside the SDK. The function must be called before calling @link #CameraDisplayRGB24 @endlink. If you use your own method for image display (do not call CameraDisplayRGB24) during secondary development, you do not need to call this function.
/// \param [in] hCamera Camera handle.
/// \param [in] hWndDisplay The handle of the display window, typically the m_hWnd member of the window.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraDisplayInit(
	CameraHandle    hCamera,
	HWND            hWndDisplay
);

/// @ingroup API_DISPLAY
/// \~chinese
/// \brief ��ʾͼ�񡣱�����ù�@link #CameraDisplayInit @endlink���г�ʼ�����ܵ��ñ�������
/// \param [in] hCamera ����ľ����
/// \param [in] pFrameBuffer ͼ���֡������
/// \param [in] pFrInfo ͼ���֡ͷ��Ϣ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Display the image. You must call @link #CameraDisplayInit @endlink before you can call this function.
/// \param [in] hCamera Camera handle.
/// \param [in] pFrameBuffer Image frame buffer
/// \param [in] pFrInfo The frame header information of the image
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraDisplayRGB24(
	CameraHandle        hCamera,
	BYTE* pFrameBuffer,
	tSdkFrameHead* pFrInfo
);

/// @ingroup API_DISPLAY
/// \~chinese
/// \brief ������ʾ��ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] iMode ��ʾģʽ���μ�@link #emSdkDisplayMode @endlink�Ķ��塣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the display mode.
/// \param [in] hCamera Camera handle.
/// \param [in] iMode Display mode, see @link #emSdkDisplayMode @endlink definition.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetDisplayMode(
	CameraHandle    hCamera,
	INT             iMode
);

/// @ingroup API_DISPLAY
/// \~chinese
/// \brief ������ʾ����ʼƫ��ֵ��������ʾģʽΪDISPLAYMODE_REALʱ��Ч��������ʾ�ؼ��Ĵ�СΪ320X240����ͼ��ĵĳߴ�Ϊ640X480����ô��iOffsetX = 160,iOffsetY = 120ʱ��ʾ���������ͼ��ľ���320X240��λ�á�
/// \param [in] hCamera ����ľ����
/// \param [in] iOffsetX ƫ�Ƶ�X���ꡣ
/// \param [in] iOffsetY  ƫ�Ƶ�Y���ꡣ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the starting offset of the display. This is valid only when the display mode is DISPLAY MODE_REAL. For example, the size of the display control is 320��240, and the size of the image is 640��480. When iOffsetX = 160 and iOffsetY = 120, the displayed area is the center 320��240 of the image.
/// \param [in] hCamera Camera handle.
/// \param [in] iOffsetX The offset's X coordinate.
/// \param [in] iOffsetY  The offset's Y coordinate.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetDisplayOffset(
	CameraHandle    hCamera,
	int             iOffsetX,
	int             iOffsetY
);

/// @ingroup API_DISPLAY
/// \~chinese
/// \brief ������ʾ�ؼ��ĳߴ硣
/// \param [in] hCamera ����ľ����
/// \param [in] iWidth ���
/// \param [in] iHeight �߶�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the size of the display control.
/// \param [in] hCamera Camera handle.
/// \param [in] iWidth width
/// \param [in] iHeight height
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetDisplaySize(
	CameraHandle    hCamera,
	INT             iWidth,
	INT             iHeight
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ�Ϊ�����Ч�ʣ�SDK��ͼ��ץȡʱ�������㿽�����ƣ�������ʵ�ʻ�����ں��е�һ����������ַ��
/// \param [in] hCamera ����ľ����
/// \param [out] pFrameInfo ͼ���֡ͷ��Ϣָ�롣
/// \param [out] pbyBuffer ����ͼ�����ݵĻ�����ָ�롣
/// \param [in] wTimes ץȡͼ��ĳ�ʱʱ�䣬��λ���롣��wTimesʱ���ڻ�δ���ͼ����ú����᷵�س�ʱ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ú����ɹ����ú󣬱������@link CameraReleaseImageBuffer @endlink�ͷŻ�����,�Ա����ں˼���ʹ�øû�������  
/// \~english
/// \brief Get a frame of image data. To improve efficiency, the SDK uses a zero-copy mechanism for image capture. This function actually obtains a buffer address in the kernel.
/// \param [in] hCamera Camera handle.
/// \param [out] pFrameInfo The header information pointer of the image.
/// \param [out] pbyBuffer Returns the buffer pointer of the image data.
/// \param [in] wTimes Timeout for grabbing an image in milliseconds. The function returns a timeout error if no image has been obtained within wTimes.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note After the function is successfully called, @link CameraReleaseImageBuffer @endlink must be called to release the buffer so that the kernel can continue to use the buffer.
MVSDK_API CameraSdkStatus __stdcall CameraGetImageBuffer(
	CameraHandle        hCamera,
	tSdkFrameHead* pFrameInfo,
	BYTE** pbyBuffer,
	UINT                wTimes
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ��ýӿڻ�õ�ͼ���������Ѿ�����ͼ��������ݡ�
/// \param [in] hCamera ����ľ����
/// \param [out] piWidth ����ָ�룬����ͼ��Ŀ�ȡ�
/// \param [out] piHeight ����ָ�룬����ͼ��ĸ߶ȡ�
/// \param [in] wTimes ץȡͼ��ĳ�ʱʱ�䣬��λ���롣��wTimesʱ���ڻ�δ���ͼ����ú����᷵�س�ʱ����
/// \return �ɹ�ʱ������֡���ݻ��������׵�ַ�����򷵻�0��
/// \note ����������Ҫ����@link CameraReleaseImageBuffer @endlink�ͷŻ�������
/// \~english
/// \brief Get a frame of image data. The image data obtained by this interface is already image-processed data.
/// \param [in] hCamera Camera handle.
/// \param [out] piWidth  returns the width of the image.
/// \param [out] piHeight Returns the height of the image.
/// \param [in] wTimes Timeout for grabbing an image in milliseconds. The function returns a timeout error if no image has been obtained within wTimes.
/// \return On success, returns the first address of the frame data buffer, otherwise it returns 0.
/// \note This function does not need to call @link CameraReleaseImageBuffer @endlink to release the buffer.
MVSDK_API unsigned char* __stdcall CameraGetImageBufferEx(
	CameraHandle        hCamera,
	INT* piWidth,
	INT* piHeight,
	UINT                wTimes
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ץ��һ��ͼ�񵽻������С���������ץ��ģʽ�������Զ��л���ץ��ģʽ�ķֱ��ʽ���ͼ�񲶻�
/// \param [in] hCamera ����ľ����
/// \param [out] pFrameInfo ͼ���֡ͷ��Ϣָ�롣
/// \param [out] pbyBuffer ����ͼ�����ݵĻ�����ָ�롣
/// \param [in] wTimes ץȡͼ��ĳ�ʱʱ�䣬��λ���롣��wTimesʱ���ڻ�δ���ͼ����ú����᷵�س�ʱ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ú����ɹ����ú󣬱������@link CameraReleaseImageBuffer @endlink�ͷŻ�����,�Ա����ں˼���ʹ�øû�������  
/// \warning ���������ܻ���зֱ����л������Ч�ʻ��@link #CameraGetImageBuffer @endlink�͡����û���л��ֱ���ץ�ĵ�������ʹ��@link #CameraGetImageBuffer @endlink��
/// \~english
/// \brief Take an image into the buffer. The camera will enter snap mode and automatically switch to snap mode resolution for image capture.
/// \param [in] hCamera Camera handle.
/// \param [out] pFrameInfo The header information pointer of the image.
/// \param [out] pbyBuffer Returns the buffer pointer of the image data.
/// \param [in] wTimes Timeout for grabbing an image in milliseconds. The function returns a timeout error if no image has been obtained within wTimes.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note After the function is successfully called, @link CameraReleaseImageBuffer @endlink must be called to release the buffer so that the kernel can continue to use the buffer.
/// \warning This function may switch the resolution, so the efficiency will be lower than @link #CameraGetImageBuffer @endlink. If you do not need to switch resolution capture, use @link #CameraGetImageBuffer @endlink.
MVSDK_API CameraSdkStatus __stdcall CameraSnapToBuffer(
	CameraHandle        hCamera,
	tSdkFrameHead* pFrameInfo,
	BYTE** pbyBuffer,
	UINT                wTimes
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ץ��һ��JPEG��ʽͼ���ļ��С�(���������Ӳ��֧�ִ˹���)
/// \param [in] hCamera ����ľ����
/// \param [in] lpszFileName ͼƬ�����ļ�����·����
/// \param [in] byQuality ͼ�񱣴���������ӣ���Χ1��100��
/// \param [in] wTimes ץȡͼ��ĳ�ʱʱ�䣬��λ���롣��wTimesʱ���ڻ�δ���ͼ����ú����᷵�س�ʱ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Capture a JPEG format image into the file. (Only some camera hardware supports this function)
/// \param [in] hCamera Handle of the camera.
/// \param [in] lpszFileName The full path of the image file.
/// \param [in] byQuality The quality factor of image saving, ranging from 1 to 100.
/// \param [in] wTimes The timeout period for capturing images, in milliseconds. If the image is not obtained within wTimes, the function will return a timeout error.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSnapJpegToFile(
	CameraHandle    hCamera,
	char const* lpszFileName,
	BYTE            byQuality,
	UINT            wTimes
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief �ͷ���@link #CameraGetImageBuffer @endlink��õĻ�������
/// \param [in] hCamera ����ľ����
/// \param [in] pbyBuffer ֡��������ַ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Releases the buffer obtained by @link #CameraGetImageBuffer @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] pbyBuffer Frame buffer address.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraReleaseImageBuffer(
	CameraHandle    hCamera,
	BYTE* pbyBuffer
);

/// @ingroup API_PLAY_CTRL
/// \~chinese
/// \brief ��������빤��ģʽ����ʼ��������������͵�ͼ�����ݡ�
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Put the camera into working mode and start receiving image data from the camera.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraPlay(
	CameraHandle hCamera
);

/// @ingroup API_PLAY_CTRL
/// \~chinese
/// \brief �����������ͣģʽ�����������������ͼ�����ݣ�ͬʱҲ�ᷢ�������������ͣ������ͷŴ��������ͣģʽ�£����Զ�����Ĳ����������ã���������Ч��
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Put the camera into pause mode, do not receive image data from the camera, and also send a command to pause the camera output and release the transmission bandwidth. In pause mode, camera parameters can be configured and take effect immediately.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraPause(
	CameraHandle hCamera
);

/// @ingroup API_PLAY_CTRL
/// \~chinese
/// \brief ���������ֹͣ״̬��һ���Ƿ���ʼ��ʱ���øú������ú��������ã������ٶ�����Ĳ����������á�
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Let the camera enter the stop state. Generally, this function is called when deinitializing. The function is called and the camera parameters cannot be configured.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraStop(
	CameraHandle hCamera
);

/// @ingroup API_RECORD
/// \~chinese
/// \brief ��ʼ��һ��¼��
/// \param [in] hCamera ����ľ����
/// \param [in] iFormat ¼��ĸ�ʽ��0:��ѹ��  1:MSCV��ʽѹ��  4:H264��
/// \param [in] pcSavePath ¼���ļ������·����
/// \param [in] b2GLimit ���ΪTRUE,���ļ�����2Gʱ�Զ��ָ������δʵ�֣�
/// \param [in] dwQuality ¼����������ӣ�Խ��������Խ�á���Χ1��100.
/// \param [in] iFrameRate ¼���֡�ʡ������趨�ı�ʵ�ʲɼ�֡�ʴ������Ͳ���©֡��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Initialize a video.
/// \param [in] hCamera Camera handle.
/// \param [in] iFormat Encoding format (0: no compression   1: MSCV compression   4: H264)
/// \param [in] pcSavePath The path to save the video file.
/// \param [in] b2GLimit If TRUE, the file is automatically split when it is larger than 2G. (Function not implemented)
/// \param [in] dwQuality The larger the quality factor of the video, the better the quality. Range 1 to 100.
/// \param [in] iFrameRate The frame rate of the video. It is recommended to set a larger frame rate than the actual acquisition so that no frames are missed.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraInitRecord(
	CameraHandle    hCamera,
	int             iFormat,
	char* pcSavePath,
	BOOL            b2GLimit,
	DWORD           dwQuality,
	int             iFrameRate
);

/// @ingroup API_RECORD
/// \~chinese
/// \brief ��������¼�񡣵�@link #CameraInitRecord @endlink�󣬿���ͨ���ú���������һ��¼�񣬲�����ļ����������
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief End this video. After @link #CameraInitRecord @endlink, you can use this function to end a video and complete the file save operation.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraStopRecord(
	CameraHandle    hCamera
);

/// @ingroup API_RECORD
/// \~chinese
/// \brief ��һ֡���ݴ���¼�����С��������ǵ�֡ͷ��Ϣ��Я����ͼ��ɼ���ʱ�����Ϣ�����¼����Ծ�׼��ʱ��ͬ����������֡�ʲ��ȶ���Ӱ�졣
/// \param [in] hCamera ����ľ����
/// \param [in] pbyImageBuffer ͼ������ݻ�������
/// \param [in] pFrInfo ͼ���֡ͷ��Ϣ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief One frame of data is stored in the video stream. Since our frame header information carries the timestamp information of the image acquisition, the recording can be accurately time synchronized without being affected by the instability of the frame rate.
/// \param [in] hCamera Camera handle.
/// \param [in] pbyImageBuffer Image data buffer.
/// \param [in] pFrInfo The frame header information of the image.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraPushFrame(
	CameraHandle    hCamera,
	BYTE* pbyImageBuffer,
	tSdkFrameHead* pFrInfo
);

/// @ingroup API_SAVE_IMAGE
/// \~chinese
/// \brief ��ͼ�񻺳��������ݱ����ͼƬ�ļ���
/// \param [in] hCamera ����ľ����
/// \param [in] lpszFileName ͼƬ�����ļ�����·����
/// \param [in] pbyImageBuffer ͼ������ݻ�������
/// \param [in] pFrInfo ͼ���֡ͷ��Ϣ��
/// \param [in] byFileType ͼ�񱣴�ĸ�ʽ��ȡֵ��Χ�μ�@link #emSdkFileType @endlink�Ķ��塣
/// \param [in] byQuality ͼ�񱣴���������ӣ���������ΪJPG��ʽʱ�ò�����Ч����Χ1��100�������ʽ����д��0��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note Ŀǰ֧�� BMP��JPG��PNG��RAW���ָ�ʽ������RAW��ʾ��������ԭʼ���ݣ�����RAW��ʽ�ļ�Ҫ��pbyImageBuffer��pFrInfo����@link #CameraGetImageBuffer @endlink��õ����ݣ�����δ��@link #CameraImageProcess @endlinkת����BMP��ʽ����֮�����Ҫ�����BMP��JPG����PNG��ʽ����pbyImageBuffer��pFrInfo����@link #CameraImageProcess @endlink������RGB��ʽ���ݡ������÷����Բο�Advanced�����̡�   
/// \~english
/// \brief Save the image buffer data as a picture file.
/// \param [in] hCamera Camera handle.
/// \param [in] lpszFileName The picture saves the full path to the file.
/// \param [in] pbyImageBuffer Image data buffer.
/// \param [in] pFrInfo The frame header information of the image.
/// \param [in] byFileType Image save format. See the definition of @link #emSdkFileType @endlink for the range of values.
/// \param [in] byQuality The quality factor of the saved image. This parameter is valid only when saving in JPG format. The range is from 1 to 100. The rest of the format can be written as 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Currently supports BMP, JPG, PNG, RAW four formats. Among them, RAW represents the raw data output by the camera. Saving RAW format files requires pbyImageBuffer and pFrInfo to be obtained by @link #CameraGetImageBuffer @endlink, and without @link #CameraImageProcess @endlink converting to BMP format; otherwise, if you want to save to BMP JPG or PNG format, pbyImageBuffer and pFrInfo are RGB format data processed by @link #CameraImageProcess @endlink. Specific usage can refer to Advanced's routines.
MVSDK_API CameraSdkStatus __stdcall CameraSaveImage(
	CameraHandle    hCamera,
	char* lpszFileName,
	BYTE* pbyImageBuffer,
	tSdkFrameHead* pFrInfo,
	UINT            byFileType,
	BYTE            byQuality
);

/// @ingroup API_SAVE_IMAGE
/// \~chinese
/// \brief ��ͼ�񻺳��������ݱ����ͼƬ�ļ���
/// \param [in] hCamera ����ľ����
/// \param [in] lpszFileName ͼƬ�����ļ�����·����
/// \param [in] pbyImageBuffer ͼ������ݻ�������
/// \param [in] uImageFormat 0:8 BIT gray   1:rgb24    2:rgba32    3:bgr24    4:bgra32
/// \param [in] iWidth ͼƬ���
/// \param [in] iHeight ͼƬ�߶�
/// \param [in] byFileType ͼ�񱣴�ĸ�ʽ��ȡֵ��Χ�μ�@link #emSdkFileType @endlink�Ķ��塣
/// \param [in] byQuality ͼ�񱣴���������ӣ���������ΪJPG��ʽʱ�ò�����Ч����Χ1��100�������ʽ����д��0��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ��@link #CameraSaveImage @endlink��ͬ
/// \~english
/// \brief Save the image buffer data as a picture file.
/// \param [in] hCamera Camera handle.
/// \param [in] lpszFileName The picture saves the full path to the file.
/// \param [in] pbyImageBuffer Image data buffer.
/// \param [in] uImageFormat 0:8 BIT gray   1:rgb24    2:rgba32    3:bgr24    4:bgra32
/// \param [in] iWidth width
/// \param [in] iHeight height
/// \param [in] byFileType Image save format. See the definition of @link #emSdkFileType @endlink for the range of values.
/// \param [in] byQuality The quality factor of the saved image. This parameter is valid only when saving in JPG format. The range is from 1 to 100. The rest of the format can be written as 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Same as @link #CameraSaveImage @endlink
MVSDK_API CameraSdkStatus __stdcall CameraSaveImageEx(
	CameraHandle    hCamera,
	char* lpszFileName,
	BYTE* pbyImageBuffer,
	UINT			uImageFormat,
	int				iWidth,
	int				iHeight,
	UINT            byFileType,
	BYTE            byQuality
);

/// @ingroup API_ROI
/// \~chinese
/// \brief ��õ�ǰԤ���ķֱ��ʡ�
/// \param [in] hCamera ����ľ����
/// \param [out] psCurVideoSize ���ص�ǰ�ķֱ��ʡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the current preview resolution.
/// \param [in] hCamera Camera handle.
/// \param [out] psCurVideoSize Returns the current resolution.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetImageResolution(
	CameraHandle            hCamera,
	tSdkImageResolution* psCurVideoSize
);

/// @ingroup API_ROI
/// \~chinese
/// \brief ��õ�ǰԤ���ķֱ��ʡ�
/// \param [in] hCamera ����ľ����
/// \param [out] iIndex		   �����ţ�[0,N]��ʾԤ��ķֱ���(N ΪԤ��ֱ��ʵ���������һ�㲻����20),OXFF ��ʾ�Զ���ֱ���(ROI)
/// \param [out] acDescription �÷ֱ��ʵ�������Ϣ����Ԥ��ֱ���ʱ����Ϣ��Ч���Զ���ֱ��ʿɺ��Ը���Ϣ
/// \param [out] Mode		   0: ��ͨģʽ     1��Sum       2��Average        3��Skip        4��Resample
/// \param [out] ModeSize	   ��ͨģʽ�º��ԣ���1λ��ʾ2X2 �ڶ�λ��ʾ3X3 ...
/// \param [out] x			   ˮƽƫ��
/// \param [out] y			   ��ֱƫ��
/// \param [out] width		   ��
/// \param [out] height		   ��
/// \param [out] ZoomWidth     �������ʱ���ſ�ȣ�0��ʾ������ 
/// \param [out] ZoomHeight    �������ʱ���Ÿ߶ȣ�0��ʾ������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the current preview resolution.
/// \param [in] hCamera Camera handle.
/// \param [out] iIndex Index number, [0,N] indicates the default resolution (N is the maximum number of preset resolutions, generally no more than 20), OXFF indicates custom resolution (ROI)
/// \param [out] acDescription Descriptive information for this resolution. This information is valid only when the resolution is preset. Custom resolution ignores this information
/// \param [out] Mode 0: Normal Mode 1:Sum 2:Average 3:Skip 4:Resample
/// \param [out] ModeSize ignored in normal mode, the first bit represents 2X2 the second bit represents 3X3 ...
/// \param [out] x horizontal offset
/// \param [out] y vertical offset
/// \param [out] width width
/// \param [out] height high
/// \param [out] ZoomWidth Scale width when final output, 0 means not zoom
/// \param [out] ZoomHeight Scales the height of the final output, 0 means no scaling
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetImageResolutionEx(
	CameraHandle            hCamera,
	int* iIndex,
	char					acDescription[32],
	int* Mode,
	UINT* ModeSize,
	int* x,
	int* y,
	int* width,
	int* height,
	int* ZoomWidth,
	int* ZoomHeight
);

/// @ingroup API_ROI
/// \~chinese
/// \brief ����Ԥ���ķֱ��ʡ�
/// \param [in] hCamera ����ľ����
/// \param [in] pImageResolution �·ֱ��ʡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the resolution of the preview.
/// \param [in] hCamera Camera handle.
/// \param [in] pImageResolution New resolution.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetImageResolution(
	CameraHandle            hCamera,
	tSdkImageResolution* pImageResolution
);

/// @ingroup API_ROI
/// \~chinese
/// \brief ��õ�ǰԤ���ķֱ��ʡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iIndex		   �����ţ�[0,N]��ʾԤ��ķֱ���(N ΪԤ��ֱ��ʵ���������һ�㲻����20),OXFF ��ʾ�Զ���ֱ���(ROI)
/// \param [in] Mode		   0: ��ͨģʽ     1��Sum       2��Average        3��Skip        4��Resample
/// \param [in] ModeSize	   ��ͨģʽ�º��ԣ���1λ��ʾ2X2 �ڶ�λ��ʾ3X3 ...
/// \param [in] x			   ˮƽƫ��
/// \param [in] y			   ��ֱƫ��
/// \param [in] width		   ��
/// \param [in] height		   ��
/// \param [in] ZoomWidth     �������ʱ���ſ�ȣ�0��ʾ������ 
/// \param [in] ZoomHeight    �������ʱ���Ÿ߶ȣ�0��ʾ������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the current preview resolution.
/// \param [in] hCamera Camera handle.
/// \param [in] iIndex Index number, [0,N] indicates the default resolution (N is the maximum number of preset resolutions, generally no more than 20), OXFF indicates custom resolution (ROI)
/// \param [in] Mode 0: Normal Mode 1:Sum 2:Average 3:Skip 4:Resample
/// \param [in] ModeSize ignored in normal mode, the first bit represents 2X2 the second bit represents 3X3 ...
/// \param [in] x horizontal offset
/// \param [in] y vertical offset
/// \param [in] width width
/// \param [in] height high
/// \param [in] ZoomWidth Scale width when final output, 0 means not zoom
/// \param [in] ZoomHeight Scales the height of the final output, 0 means no scaling
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetImageResolutionEx(
	CameraHandle            hCamera,
	int						iIndex,
	int						Mode,
	UINT					ModeSize,
	int						x,
	int						y,
	int						width,
	int						height,
	int						ZoomWidth,
	int						ZoomHeight
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������ǰ���ԭʼ���ݵĸ�ʽ�����š�
/// \param [in] hCamera ����ľ����
/// \param [out] piMediaType ���ص�ǰ��ʽ���͵������š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ��@link #tSdkCameraCapbility.pMediaTypeDesc @endlink��Ա�У����������ʽ���������֧�ֵĸ�ʽ��piMediaType��ָ��������ţ����Ǹ�����������š�
/// \~english
/// \brief Gets the format index number of the camera's current output raw data.
/// \param [in] hCamera Camera handle.
/// \param [out] piMediaType Returns the index of the current format type.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note In the @link #tSdkCameraCapbility.pMediaTypeDesc @endlink member, the format supported by the camera is saved as an array. The index number pointed to by piMediaType is the index number of the array.
MVSDK_API CameraSdkStatus __stdcall CameraGetMediaType(
	CameraHandle    hCamera,
	INT* piMediaType
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ������������ԭʼ���ݸ�ʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] iMediaType �¸�ʽ���͵������š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ��@link #CameraGetMediaType @endlink��ͬ��
/// \~english
/// \brief Sets the camera's output raw data format.
/// \param [in] hCamera Camera handle.
/// \param [in] iMediaType The index number of the new format type.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Same as @link #CameraGetMediaType @endlink.
MVSDK_API CameraSdkStatus __stdcall CameraSetMediaType(
	CameraHandle    hCamera,
	INT             iMediaType
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡRAW���ݵ������Чλ��
/// \param [in] hCamera ����ľ����
/// \param [out] pMaxAvailBits	����RAW�������Чλ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the maximum number of significant bits of RAW data
/// \param [in] hCamera Camera handle.
/// \param [out] pMaxAvailBits	returns the maximum number of significant bits of RAW data
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetRawMaxAvailBits(
	CameraHandle    hCamera,
	int* pMaxAvailBits
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ����RAW���ݵ������ʼλ
/// \param [in] hCamera ����ľ����
/// \param [in] startBit ��ʼBIT��Ĭ�������8λ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the output start bit of RAW data
/// \param [in] hCamera Camera handle.
/// \param [in] startBit Start BIT (The high 8 bits are output by default)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetRawStartBit(
	CameraHandle    hCamera,
	int             startBit
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡRAW���ݵ������ʼλ
/// \param [in] hCamera ����ľ����
/// \param [out] startBit ��ʼBIT
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the output start bit of RAW data
/// \param [in] hCamera Camera handle.
/// \param [out] startBit Start BIT
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetRawStartBit(
	CameraHandle    hCamera,
	int* startBit
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��������ع��ģʽ���Զ������ֶ���
/// \param [in] hCamera ����ľ����
/// \param [in] bAeState TRUE:�Զ��ع⣻FALSE:�ֶ��ع⡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera exposure mode. Automatic or manual.
/// \param [in] hCamera Camera handle.
/// \param [in] bAeState TRUE: Auto exposure; FALSE: Manual exposure.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeState(
	CameraHandle    hCamera,
	BOOL            bAeState
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��������ǰ���ع�ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pAeState �����Զ��ع��ʹ��״̬��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's current exposure mode.
/// \param [in] hCamera Camera handle.
/// \param [out] pAeState Returns the auto exposure's enable state.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeState(
	CameraHandle    hCamera,
	BOOL* pAeState
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ����ͼ��Ĵ�����񻯲�����
/// \param [in] hCamera ����ľ����
/// \param [in] iSharpness �񻯲�����һ����[0,100]��0��ʾ�ر��񻯴���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the sharpening parameters for the processing of the image.
/// \param [in] hCamera Camera handle.
/// \param [in] iSharpness Sharpen parameter, generally [0,100], 0 means close sharpening.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetSharpness(
	CameraHandle    hCamera,
	int             iSharpness
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��ȡ��ǰ���趨ֵ��
/// \param [in] hCamera ����ľ����
/// \param [out] piSharpness ���ص�ǰ�趨���񻯵��趨ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gets the current sharpening setting.
/// \param [in] hCamera Camera handle.
/// \param [out] piSharpness Returns the currently set sharpened setting.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetSharpness(
	CameraHandle    hCamera,
	int* piSharpness
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��������Ĳ��任ģʽLUTģʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] emLutMode ����ο�@link #emSdkLutMode @endlink���͡�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera's lookup table transformation mode LUT mode.
/// \param [in] hCamera Camera handle.
/// \param [in] emLutMode Defines the reference @link #emSdkLutMode @endlink type.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLutMode(
	CameraHandle    hCamera,
	int             emLutMode
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief �������Ĳ��任ģʽLUTģʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pemLutMode ���ص�ǰLUTģʽ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Obtain the look-up table conversion mode LUT mode of the camera.
/// \param [in] hCamera Camera handle.
/// \param [out] pemLutMode Returns the current LUT mode.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLutMode(
	CameraHandle    hCamera,
	int* pemLutMode
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ѡ��Ԥ��LUTģʽ�µ�LUT��
/// \param [in] hCamera ����ľ����
/// \param [in] iSel ��������š���ĸ�����@link #tSdkCameraCapbility.iPresetLut @endlink��á�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ������ʹ��@link #CameraSetLutMode @endlink��LUTģʽ����ΪԤ��ģʽ��
/// \~english
/// \brief Select the LUT table in the preset LUT mode.
/// \param [in] hCamera Camera handle.
/// \param [in] iSel The index number of the lut table. The number of tables is obtained by @link #tSdkCameraCapbility.iPresetLut @endlink.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Must use @link #CameraSetLutMode @endlink to set LUT mode to preset mode.
MVSDK_API CameraSdkStatus __stdcall CameraSelectLutPreset(
	CameraHandle    hCamera,
	int             iSel
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���Ԥ��LUTģʽ�µ�LUT�������š�
/// \param [in] hCamera ����ľ����
/// \param [out] piSel  ���ر�������š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The LUT table index number in the preset LUT mode is obtained.
/// \param [in] hCamera Camera handle.
/// \param [out] piSel Returns the index number of the table.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLutPresetSel(
	CameraHandle    hCamera,
	int* piSel
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief �����Զ����LUT��
/// \param [in] hCamera ����ľ����
/// \param [in] iChannel ָ��Ҫ�趨��LUT��ɫͨ������Ϊ@link #LUT_CHANNEL_ALL @endlinkʱ������ͨ����LUT����ͬʱ�滻��@see emSdkLutChannel
/// \param [in] pLut     ָ�룬ָ��LUT��ĵ�ַ��LUT��Ϊ�޷��Ŷ��������飬�����СΪ4096���ֱ������ɫͨ����0��4096(12bit��ɫ����)��Ӧ��ӳ��ֵ�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ������ʹ��@link #CameraSetLutMode @endlink��LUTģʽ����Ϊ�Զ���ģʽ��
/// \~english
/// \brief Set up a custom LUT table.
/// \param [in] hCamera Camera handle.
/// \param [in] iChannel Specifies the LUT color channel to be set. When @link #LUT_CHANNEL_ALL @endlink, the three-channel LUTs will be replaced at the same time. @see emSdkLutChannel
/// \param [in] pLut pointer to the address of the LUT table. The LUT table is an unsigned short integer array, and the array size is 4096, which is the mapping value corresponding to the code color channel from 0 to 4096 (12 bit color accuracy).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note You must use @link #CameraSetLutMode @endlink to set the LUT mode to custom mode.
MVSDK_API CameraSdkStatus __stdcall CameraSetCustomLut(
	CameraHandle    hCamera,
	int       iChannel,
	USHORT* pLut
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��õ�ǰʹ�õ��Զ���LUT��
/// \param [in] hCamera ����ľ����
/// \param [in] iChannel ָ��Ҫ��ȡ��LUT��ɫͨ������Ϊ@link #LUT_CHANNEL_ALL @endlinkʱ�����غ�ɫͨ����LUT��@see emSdkLutChannel
/// \param [out] pLut    ָ��LUT��ĵ�ַ��LUT��Ϊ�޷��Ŷ��������飬�����СΪ4096���ֱ������ɫͨ����0��4096(12bit��ɫ����)��Ӧ��ӳ��ֵ�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the currently used custom LUT table.
/// \param [in] hCamera Camera handle.
/// \param [in] iChannel Specifies the LUT color channel to be obtained. When @link #LUT_CHANNEL_ALL @endlink, returns the LUT table of the red channel. @see emSdkLutChannel
/// \param [out] pLut points to the address of the LUT table. The LUT table is an unsigned short integer array, and the array size is 4096, which is the mapping value corresponding to the code color channel from 0 to 4096 (12 bit color accuracy).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCustomLut(
	CameraHandle    hCamera,
	int       iChannel,
	USHORT* pLut
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��������ǰ��LUT�����κ�LUTģʽ�¶����Ե���,����ֱ�۵Ĺ۲�LUT���ߵı仯��
/// \param [in] hCamera ����ľ����
/// \param [in] iChannel ָ��Ҫ��ȡ��LUT��ɫͨ������Ϊ@link #LUT_CHANNEL_ALL @endlinkʱ�����غ�ɫͨ����LUT��@see emSdkLutChannel
/// \param [out] pLut    ָ��LUT��ĵ�ַ��LUT��Ϊ�޷��Ŷ��������飬�����СΪ4096���ֱ������ɫͨ����0��4096(12bit��ɫ����)��Ӧ��ӳ��ֵ�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Obtain the camera's current LUT table, which can be called in any LUT mode, to intuitively observe changes in the LUT curve.
/// \param [in] hCamera Camera handle.
/// \param [in] iChannel Specifies the LUT color channel to be obtained. When @link #LUT_CHANNEL_ALL @endlink, returns the LUT table of the red channel. @see emSdkLutChannel
/// \param [out] pLut points to the address of the LUT table. The LUT table is an unsigned short integer array, and the array size is 4096, which is the mapping value corresponding to the code color channel from 0 to 4096 (12 bit color accuracy).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCurrentLut(
	CameraHandle    hCamera,
	int       iChannel,
	USHORT* pLut
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ���������ƽ��ģʽ����Ϊ�ֶ����Զ����ַ�ʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] bAuto TRUE�����ʾʹ���Զ�ģʽ�� FALSE�����ʾʹ���ֶ�ģʽ��ͨ������@link #CameraSetOnceWB @endlink������һ�ΰ�ƽ�⡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set camera white balance mode. Divided into manual and automatic two ways.
/// \param [in] hCamera Camera handle.
/// \param [in] bAuto TRUE to enable auto mode. FALSE indicates that using manual mode, a white balance is performed by calling @link #CameraSetOnceWB @endlink.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetWbMode(
	CameraHandle    hCamera,
	BOOL            bAuto
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ��õ�ǰ�İ�ƽ��ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pbAuto   ָ�룬����TRUE��ʾ�Զ�ģʽ��FALSEΪ�ֶ�ģʽ�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the current white balance mode.
/// \param [in] hCamera Camera handle.
/// \param [out] pbAuto pointer, return TRUE for automatic mode, FALSE for manual mode.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetWbMode(
	CameraHandle    hCamera,
	BOOL* pbAuto
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ѡ��ָ��Ԥ��ɫ��ģʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iSel Ԥ��ɫ�µ�ģʽ�����ţ���0��ʼ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����@link #CameraSetClrTempMode @endlink����ΪԤ��ģʽ��
/// \~english
/// \brief Select the specified preset color temperature mode
/// \param [in] hCamera Camera handle.
/// \param [in] iSel The mode index number of the preset color temperature, starting from 0
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Call @link #CameraSetClrTempMode @endlink set to preset mode.
MVSDK_API CameraSdkStatus __stdcall CameraSetPresetClrTemp(
	CameraHandle    hCamera,
	int             iSel
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ��õ�ǰѡ���Ԥ��ɫ��ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] piSel  ����ѡ���Ԥ��ɫ��������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the currently selected preset color temperature mode.
/// \param [in] hCamera Camera handle.
/// \param [out] piSel Returns the selected preset color temperature index number
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetPresetClrTemp(
	CameraHandle    hCamera,
	int* piSel
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief �����Զ���ɫ��ģʽ�µ���������
/// \param [in] hCamera ����ľ����
/// \param [in] iRgain  ��ɫ���棬��Χ0��400����ʾ0��4��
/// \param [in] iGgain  ��ɫ���棬��Χ0��400����ʾ0��4��
/// \param [in] iBgain  ��ɫ���棬��Χ0��400����ʾ0��4��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����@link #CameraSetClrTempMode @endlink����Ϊ�Զ���ģʽ��
/// \~english
/// \brief Set digital gain in custom color temperature mode
/// \param [in] hCamera Camera handle.
/// \param [in] iRgain Red gain, range 0 to 400, 0 to 4 times
/// \param [in] iGgain Green gain, range 0 to 400, 0 to 4 times
/// \param [in] iBgain Blue gain, range 0 to 400, 0 to 4 times
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Call @link #CameraSetClrTempMode @endlink set to custom mode.
MVSDK_API CameraSdkStatus __stdcall CameraSetUserClrTempGain(
	CameraHandle  hCamera,
	int       iRgain,
	int       iGgain,
	int       iBgain
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ����Զ���ɫ��ģʽ�µ���������
/// \param [in] hCamera ����ľ����
/// \param [out] piRgain  ָ�룬���غ�ɫ���棬��Χ0��400����ʾ0��4��
/// \param [out] piGgain  ָ�룬������ɫ���棬��Χ0��400����ʾ0��4��
/// \param [out] piBgain  ָ�룬������ɫ���棬��Χ0��400����ʾ0��4��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get digital gain in custom color temperature mode
/// \param [in] hCamera Camera handle.
/// \param [out] piRgain pointer, returning red gain, range 0 to 400, 0 to 4 times
/// \param [out] piGgain pointer, return green gain, range 0 to 400, 0 to 4 times
/// \param [out] piBgain pointer, returns blue gain, range 0 to 400, 0 to 4 times
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetUserClrTempGain(
	CameraHandle  hCamera,
	int* piRgain,
	int* piGgain,
	int* piBgain
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief �����Զ���ɫ��ģʽ�µ���ɫ����
/// \param [in] hCamera ����ľ����
/// \param [in] pMatrix ָ��һ��float[3][3]������׵�ַ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����@link #CameraSetClrTempMode @endlink����Ϊ�Զ���ģʽ��
/// \~english
/// \brief Set the color matrix in custom color temperature mode
/// \param [in] hCamera Camera handle.
/// \param [in] pMatrix points to the first address of an array of float[3][3]
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Call @link #CameraSetClrTempMode @endlink set to custom mode.
MVSDK_API CameraSdkStatus __stdcall CameraSetUserClrTempMatrix(
	CameraHandle  hCamera,
	float* pMatrix
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ����Զ���ɫ��ģʽ�µ���ɫ����
/// \param [in] hCamera ����ľ����
/// \param [out] pMatrix ָ��һ��float[3][3]������׵�ַ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the color matrix in a custom color temperature mode
/// \param [in] hCamera Camera handle.
/// \param [out] pMatrix points to the first address of an array of float[3][3]
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetUserClrTempMatrix(
	CameraHandle  hCamera,
	float* pMatrix
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ���ð�ƽ��ʱʹ�õ�ɫ��ģʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iMode ģʽ��ֻ����@link #emSdkClrTmpMode @endlink�ж����һ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ֧�ֵ�ģʽ�����֣��ֱ����Զ���Ԥ����Զ��塣
/// \note �Զ�ģʽ�£����Զ�ѡ����ʵ�ɫ��ģʽ
/// \note Ԥ��ģʽ�£���ʹ���û�ָ����ɫ��ģʽ
/// \note �Զ���ģʽ�£�ʹ���û��Զ����ɫ����������;���
/// \~english
/// \brief Color temperature mode used when setting white balance
/// \param [in] hCamera Camera handle.
/// \param [in] iMode mode, can only be defined by @link #emSdkClrTmpMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note There are three supported modes, automatic, preset, and custom.
/// \note Automatic mode will automatically select the appropriate color temperature mode
/// \note In preset mode, user-specified color temperature mode is used
/// \note custom-defined color temperature digital gain and matrix
MVSDK_API CameraSdkStatus __stdcall CameraSetClrTempMode(
	CameraHandle  hCamera,
	int       iMode
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ��ð�ƽ��ʱʹ�õ�ɫ��ģʽ���ο�@link #CameraSetClrTempMode @endlink�й����������֡�
/// \param [in] hCamera ����ľ����
/// \param [out] pimode ָ�룬����ģʽѡ�񣬲ο�@link #emSdkClrTmpMode @endlink���Ͷ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The color temperature mode used when obtaining white balance. Refer to the function description section of @link #CameraSetClrTempMode @endlink.
/// \param [in] hCamera Camera handle.
/// \param [out] pimode pointer, return mode selection, reference @link #emSdkClrTmpMode @endlink type definition
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetClrTempMode(
	CameraHandle  hCamera,
	int* pimode
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ���ֶ���ƽ��ģʽ�£����øú��������һ�ΰ�ƽ�⡣��Ч��ʱ��Ϊ���յ���һ֡ͼ������ʱ��
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief In manual white balance mode, calling this function will perform a white balance. The effective time is when the next frame of image data is received.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetOnceWB(
	CameraHandle    hCamera
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ִ��һ�κ�ƽ�����������Ҫ���֧�ֱ����ܣ�
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Performs a black balance operation. (Requires camera support for this feature)
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetOnceBB(
	CameraHandle    hCamera
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �趨�Զ��ع������Ŀ��ֵ���趨��Χ[@link #tSdkExpose.uiTargetMin @endlink, @link #tSdkExpose.uiTargetMax @endlink]
/// \param [in] hCamera ����ľ����
/// \param [in] iAeTarget ����Ŀ��ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the brightness target for auto exposure. Setting range [@link #tSdkExpose.uiTargetMin @endlink, @link #tSdkExpose.uiTargetMax @endlink]
/// \param [in] hCamera Camera handle.
/// \param [in] iAeTarget Brightness target value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeTarget(
	CameraHandle    hCamera,
	int             iAeTarget
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع������Ŀ��ֵ��
/// \param [in] hCamera ����ľ����
/// \param [out] piAeTarget ָ�룬����Ŀ��ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the auto exposure's brightness target value.
/// \param [in] hCamera Camera handle.
/// \param [out] piAeTarget pointer, return target value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeTarget(
	CameraHandle    hCamera,
	int* piAeTarget
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �趨�Զ��ع�ģʽ���ع�ʱ����ڷ�Χ
/// \param [in] hCamera ����ľ����
/// \param [in] fMinExposureTime ��С�ع�ʱ�䣨΢�룩
/// \param [in] fMaxExposureTime ����ع�ʱ�䣨΢�룩
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Setting the exposure time adjustment range of the automatic exposure mode
/// \param [in] hCamera Camera handle.
/// \param [in] fMinExposureTime Minimum exposure time (microseconds)
/// \param [in] fMaxExposureTime Maximum exposure time (microseconds)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeExposureRange(
	CameraHandle    hCamera,
	double          fMinExposureTime,
	double			fMaxExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع�ģʽ���ع�ʱ����ڷ�Χ
/// \param [in] hCamera ����ľ����
/// \param [out] fMinExposureTime ��С�ع�ʱ�䣨΢�룩
/// \param [out] fMaxExposureTime ����ع�ʱ�䣨΢�룩
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Exposure time adjustment range for automatic exposure mode
/// \param [in] hCamera Camera handle.
/// \param [out] fMinExposureTime Minimum exposure time (microseconds)
/// \param [out] fMaxExposureTime Maximum exposure time (microseconds)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeExposureRange(
	CameraHandle    hCamera,
	double* fMinExposureTime,
	double* fMaxExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �趨�Զ��ع�ģʽ��������ڷ�Χ
/// \param [in] hCamera ����ľ����
/// \param [in] iMinAnalogGain ��С����
/// \param [in] iMaxAnalogGain �������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Setting the gain adjustment range of the automatic exposure mode
/// \param [in] hCamera Camera handle.
/// \param [in] iMinAnalogGain minimum gain
/// \param [in] iMaxAnalogGain maximum gain
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeAnalogGainRange(
	CameraHandle    hCamera,
	int				iMinAnalogGain,
	int				iMaxAnalogGain
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع�ģʽ��������ڷ�Χ
/// \param [in] hCamera ����ľ����
/// \param [out] iMinAnalogGain ��С����
/// \param [out] iMaxAnalogGain �������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gain adjustment range for automatic exposure mode
/// \param [in] hCamera Camera handle.
/// \param [out] iMinAnalogGain minimum gain
/// \param [out] iMaxAnalogGain maximum gain
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeAnalogGainRange(
	CameraHandle    hCamera,
	int* iMinAnalogGain,
	int* iMaxAnalogGain
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����Զ��ع�ģʽ�ĵ�����ֵ
/// \param [in] hCamera ����ľ����
/// \param [in] iThreshold   ��� abs(Ŀ������-ͼ������) < iThreshold ��ֹͣ�Զ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the adjustment threshold for auto exposure mode
/// \param [in] hCamera Camera handle.
/// \param [in] iThreshold Stops automatic adjustment if abs (target brightness - image brightness) < iThreshold
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeThreshold(
	CameraHandle    hCamera,
	int				iThreshold
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��ȡ�Զ��ع�ģʽ�ĵ�����ֵ
/// \param [in] hCamera ����ľ����
/// \param [out] iThreshold   ��ȡ���ĵ�����ֵ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get adjustment threshold for auto exposure mode
/// \param [in] hCamera Camera handle.
/// \param [out] iThreshold Read Threshold
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeThreshold(
	CameraHandle    hCamera,
	int* iThreshold
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����ع�ʱ�䡣��λΪ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [in] fExposureTime �ع�ʱ�䣬��λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����CMOS�����������ع�ĵ�λ�ǰ�����������ģ���ˣ��ع�ʱ�䲢������΢�뼶�������ɵ������ǻᰴ��������ȡ�ᡣ�ڵ��ñ������趨�ع�ʱ��󣬽����ٵ���@link #CameraGetExposureTime @endlink�����ʵ���趨��ֵ��
/// \~english
/// \brief Set the exposure time. The unit is microseconds.
/// \param [in] hCamera Camera handle.
/// \param [in] fExposureTime Exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note For CMOS sensors, the unit of exposure is calculated in rows, so the exposure time cannot be continuously adjusted in microseconds. Instead, the entire line will be chosen. After calling this function to set the exposure time, it is recommended to call @link #CameraGetExposureTime @endlink to get the actual set value.
MVSDK_API CameraSdkStatus __stdcall CameraSetExposureTime(
	CameraHandle    hCamera,
	double          fExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���һ�е��ع�ʱ�䡣
/// \param [in] hCamera ����ľ����
/// \param [out] pfLineTime ָ�룬����һ�е��ع�ʱ�䣬��λΪ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����CMOS�����������ع�ĵ�λ�ǰ�����������ģ���ˣ��ع�ʱ�䲢������΢�뼶�������ɵ������ǻᰴ��������ȡ�ᡣ������������þ��Ƿ���CMOS����ع�һ�ж�Ӧ��ʱ�䡣
/// \~english
/// \brief Get a line of exposure time.
/// \param [in] hCamera Camera handle.
/// \param [out] pfLineTime returns the exposure time of one line in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note For CMOS sensors, the unit of exposure is calculated in rows, so the exposure time cannot be continuously adjusted in microseconds. Instead, the entire line will be chosen. The function of this function is to return the CMOS camera exposure one line corresponding time.
MVSDK_API CameraSdkStatus __stdcall CameraGetExposureLineTime(
	CameraHandle    hCamera,
	double* pfLineTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���������ع�ʱ�䡣
/// \param [in] hCamera ����ľ����
/// \param [out] pfExposureTime   ָ�룬���ص�ǰ���ع�ʱ�䣬��λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetExposureTime
/// \~english
/// \brief Get camera exposure time.
/// \param [in] hCamera Camera handle.
/// \param [out] pfExposureTime returns the current exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetExposureTime
MVSDK_API CameraSdkStatus __stdcall CameraGetExposureTime(
	CameraHandle    hCamera,
	double* pfExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���������ع�ʱ�䷶Χ
/// \param [in] hCamera ����ľ����
/// \param [out] pfMin		ָ�룬�����ع�ʱ�����Сֵ����λ΢�롣
/// \param [out] pfMax		ָ�룬�����ع�ʱ������ֵ����λ΢�롣
/// \param [out] pfStep		ָ�룬�����ع�ʱ��Ĳ���ֵ����λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get camera exposure time range
/// \param [in] hCamera Camera handle.
/// \param [out] pfMin Returns the minimum exposure time in microseconds.
/// \param [out] pfMax Returns the maximum exposure time in microseconds.
/// \param [out] pfStep Returns the exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetExposureTimeRange(
	CameraHandle    hCamera,
	double* pfMin,
	double* pfMax,
	double* pfStep
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���ö����ع�ʱ�䡣��λΪ΢�롣(�˹��ܽ��������֧��)
/// \param [in] hCamera ����ľ����
/// \param [in] index �ع�������
/// \param [in] fExposureTime �ع�ʱ�䣬��λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����CMOS�����������ع�ĵ�λ�ǰ�����������ģ���ˣ��ع�ʱ�䲢������΢�뼶�������ɵ������ǻᰴ��������ȡ�ᡣ�ڵ��ñ������趨�ع�ʱ��󣬽����ٵ���@link #CameraGetMultiExposureTime @endlink�����ʵ���趨��ֵ��
/// \~english
/// \brief Set the multiple exposure time. The unit is microseconds. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [in] index Exposure index.
/// \param [in] fExposureTime Exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note For CMOS sensors, the unit of exposure is calculated in rows, so the exposure time cannot be continuously adjusted in microseconds. Instead, the entire line will be chosen. After calling this function to set the exposure time, it is recommended to call @link #CameraGetMultiExposureTime @endlink to get the actual set value.
MVSDK_API CameraSdkStatus __stdcall CameraSetMultiExposureTime(
	CameraHandle    hCamera,
	int				index,
	double          fExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��ȡ�����ع�ʱ�䡣��λΪ΢�롣(�˹��ܽ��������֧��)
/// \param [in] hCamera ����ľ����
/// \param [in] index �ع�������
/// \param [out] fExposureTime �����ع�ʱ�䣬��λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the multiple exposure time. The unit is microseconds. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [in] index Exposure index.
/// \param [out] fExposureTime Returns exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetMultiExposureTime(
	CameraHandle    hCamera,
	int				index,
	double* fExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���ö����ع�ʹ�ܸ�����(�˹��ܽ��������֧��)
/// \param [in] hCamera ����ľ����
/// \param [in] count ʹ�ܸ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the number of multiple exposure enable. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [in] count The number of exposures enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetMultiExposureCount(
	CameraHandle    hCamera,
	int				count
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��ȡ�����ع�ʹ�ܸ�����(�˹��ܽ��������֧��)
/// \param [in] hCamera ����ľ����
/// \param [out] count ʹ�ܸ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the number of multiple exposure enable. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [out] count The number of exposures enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetMultiExposureCount(
	CameraHandle    hCamera,
	int* count
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��ȡ�����ع������ع������(�˹��ܽ��������֧��)
/// \param [in] hCamera ����ľ����
/// \param [out] max_count ֧�ֵ�����ع������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the maximum number of exposures for multiple exposures. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [out] max_count The maximum number of exposures supported.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetMultiExposureMaxCount(
	CameraHandle    hCamera,
	int* max_count
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���������ͼ��ģ������ֵ��
/// \param [in] hCamera ����ľ����
/// \param [in] iAnalogGain �趨��ģ������ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ��ֵ����@link #tSdkExpose.fAnalogGainStep @endlink���͵õ�ʵ�ʵ�ͼ���źŷŴ�����
/// \note @link CameraSetAnalogGainX @endlink�ԷŴ���Ϊ��λ��
/// \~english
/// \brief Set the camera's image analog gain value.
/// \param [in] hCamera Camera handle.
/// \param [in] iAnalogGain gain value set
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note This value is multiplied by @link #tSdkExpose.fAnalogGainStep @endlink to get the actual image signal magnification.
/// \note @link CameraSetAnalogGainX @endlink takes the magnification as the unit.
MVSDK_API CameraSdkStatus __stdcall CameraSetAnalogGain(
	CameraHandle    hCamera,
	INT             iAnalogGain
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���ͼ���źŵ�ģ������ֵ��
/// \param [in] hCamera ����ľ����
/// \param [out] piAnalogGain ָ�룬���ص�ǰ��ģ������ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note CameraGetAnalogGainX�ԷŴ���Ϊ��λ��
/// \see CameraSetAnalogGain
/// \~english
/// \brief Obtain the analog gain value of the image signal.
/// \param [in] hCamera Camera handle.
/// \param [out] piAnalogGain Returns the current analog gain value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note CameraGetAnalogGainX takes the magnification as the unit.
/// \see CameraSetAnalogGain
MVSDK_API CameraSdkStatus __stdcall CameraGetAnalogGain(
	CameraHandle    hCamera,
	INT* piAnalogGain
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���������ģ������Ŵ�����
/// \param [in] hCamera ����ľ����
/// \param [in] fGain �趨��ģ������Ŵ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the image gain magnification of the camera.
/// \param [in] hCamera Camera handle.
/// \param [in] fGain Gain magnification.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAnalogGainX(
	CameraHandle    hCamera,
	float    		fGain
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ���ͼ���źŵ�ģ������Ŵ�����
/// \param [in] hCamera ����ľ����
/// \param [out] pfGain ָ�룬���ص�ǰ��ģ������Ŵ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetAnalogGainX
/// \~english
/// \brief Obtain the gain magnification of the image signal.
/// \param [in] hCamera Camera handle.
/// \param [out] pfGain pointer, returns the current gain magnification.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetAnalogGainX
MVSDK_API CameraSdkStatus __stdcall CameraGetAnalogGainX(
	CameraHandle    hCamera,
	float* pfGain
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��������ģ������Ŵ���ȡֵ��Χ
/// \param [in] hCamera		����ľ����
/// \param [out] pfMin		ָ�룬������С������
/// \param [out] pfMax		ָ�룬�����������
/// \param [out] pfStep		ָ�룬���ز���ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the value range of the camera's gain magnification
/// \param [in] hCamera		Camera handle.
/// \param [out] pfMin		pointer, returns the minimum multiple.
/// \param [out] pfMax		pointer, returns the maximum multiple.
/// \param [out] pfStep		pointer, returns the step value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAnalogGainXRange(
	CameraHandle	hCamera,
	float* pfMin,
	float* pfMax,
	float* pfStep
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ����ͼ����������档
/// \param [in] hCamera ����ľ����
/// \param [in] iRGain   ��ɫͨ��������ֵ�� 
/// \param [in] iGGain   ��ɫͨ��������ֵ��
/// \param [in] iBGain   ��ɫͨ��������ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �趨��Χ��@link #tRgbGainRange @endlink��Ա������ʵ�ʵķŴ������趨ֵ/100��
/// \~english
/// \brief Set the digital gain of the image.
/// \param [in] hCamera Camera handle.
/// \param [in] iRGain The gain value of the red channel.
/// \param [in] iGGain Gain value of green channel.
/// \param [in] iBGain The gain value of the blue channel.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note The set scope is described by the @link #tRgbGainRange @endlink member. The actual magnification is the setting /100.
MVSDK_API CameraSdkStatus __stdcall CameraSetGain(
	CameraHandle    hCamera,
	int             iRGain,
	int             iGGain,
	int             iBGain
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ���ͼ������������档
/// \param [in] hCamera ����ľ����
/// \param [out] piRGain  ָ�룬���غ�ɫͨ������������ֵ��
/// \param [out] piGGain    ָ�룬������ɫͨ������������ֵ��
/// \param [out] piBGain    ָ�룬������ɫͨ������������ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetGain
/// \~english
/// \brief Get the digital gain of image processing.
/// \param [in] hCamera Camera handle.
/// \param [out] piRGain Returns the digital gain value of the red channel.
/// \param [out] piGGain Returns the digital gain value of the green channel.
/// \param [out] piBGain Returns the digital gain value of the blue channel.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetGain
MVSDK_API CameraSdkStatus __stdcall CameraGetGain(
	CameraHandle    hCamera,
	int* piRGain,
	int* piGGain,
	int* piBGain
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief �趨LUT��̬����ģʽ�µ�Gammaֵ��
/// \param [in] hCamera ����ľ����
/// \param [in] iGamma     Ҫ�趨��Gammaֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �趨��ֵ�����ϱ�����SDK�ڲ�������ֻ�е�������ڶ�̬�������ɵ�LUTģʽʱ���Ż���Ч����ο�@link #CameraSetLutMode @endlink�ĺ���˵�����֡�
/// \~english
/// \brief Set the gamma value in LUT dynamic generation mode.
/// \param [in] hCamera Camera handle.
/// \param [in] iGamma The gamma to be set.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note The set value will be stored in the SDK immediately, but it will only take effect when the camera is in LUT mode generated by dynamic parameters. Please refer to the function description part of @link #CameraSetLutMode @endlink.
MVSDK_API CameraSdkStatus __stdcall CameraSetGamma(
	CameraHandle    hCamera,
	int             iGamma
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���LUT��̬����ģʽ�µ�Gammaֵ
/// \param [in] hCamera ����ľ����
/// \param [out] piGamma    ָ�룬���ص�ǰ��Gammaֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetGamma
/// \~english
/// \brief Get gamma value in LUT dynamic generation mode
/// \param [in] hCamera Camera handle.
/// \param [out] piGamma Returns the current gamma value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetGamma
MVSDK_API CameraSdkStatus __stdcall CameraGetGamma(
	CameraHandle    hCamera,
	int* piGamma
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief �趨LUT��̬����ģʽ�µĶԱȶ�ֵ��
/// \param [in] hCamera ����ľ����
/// \param [in] iContrast  �趨�ĶԱȶ�ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �趨��ֵ�����ϱ�����SDK�ڲ�������ֻ�е�������ڶ�̬�������ɵ�LUTģʽʱ���Ż���Ч����ο�@link #CameraSetLutMode @endlink�ĺ���˵�����֡�
/// \~english
/// \brief Sets the contrast value in LUT dynamic generation mode.
/// \param [in] hCamera Camera handle.
/// \param [in] iContrast Contrast value set by iContrast.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note The set value will be stored in the SDK immediately, but it will only take effect when the camera is in LUT mode generated by dynamic parameters. Please refer to the function description part of @link #CameraSetLutMode @endlink.
MVSDK_API CameraSdkStatus __stdcall CameraSetContrast(
	CameraHandle    hCamera,
	int             iContrast
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���LUT��̬����ģʽ�µĶԱȶ�ֵ��
/// \param [in] hCamera ����ľ����
/// \param [out] piContrast ָ�룬���ص�ǰ�ĶԱȶ�ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetContrast
/// \~english
/// \brief Get the contrast value in LUT dynamic generation mode.
/// \param [in] hCamera Camera handle.
/// \param [out] piContrast Returns the current contrast value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetContrast
MVSDK_API CameraSdkStatus __stdcall CameraGetContrast(
	CameraHandle    hCamera,
	int* piContrast
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief �趨ͼ����ı��Ͷȡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iSaturation  �趨�ı��Ͷ�ֵ�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �Ժڰ������Ч���趨��Χ��@link #tSaturationRange @endlink��á�100��ʾԭʼɫ�ȣ�����ǿ��
/// \~english
/// \brief Sets the saturation of image processing.
/// \param [in] hCamera Camera handle.
/// \param [in] iSaturation saturation value.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note is not valid for black and white cameras. The setting range is obtained by @link #tSaturationRange @endlink. 100 represents the original color and is not enhanced.
MVSDK_API CameraSdkStatus __stdcall CameraSetSaturation(
	CameraHandle    hCamera,
	int             iSaturation
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���ͼ����ı��Ͷȡ�
/// \param [in] hCamera ����ľ����
/// \param [out] piSaturation ָ�룬���ص�ǰͼ����ı��Ͷ�ֵ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetSaturation
/// \~english
/// \brief Get image processing saturation.
/// \param [in] hCamera Camera handle.
/// \param [out] piSaturation Returns the saturation value of the current image processing.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetSaturation
MVSDK_API CameraSdkStatus __stdcall CameraGetSaturation(
	CameraHandle    hCamera,
	int* piSaturation
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���ò�ɫתΪ�ڰ׹��ܵ�ʹ�ܡ�
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable   TRUE����ʾ����ɫͼ��תΪ�ڰס�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the color to mono function enable.
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable TRUE to change the color image to black and white.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetMonochrome(
	CameraHandle    hCamera,
	BOOL            bEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��ò�ɫת���ڰ׹��ܵ�ʹ��״����
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable   ָ�롣����TRUE��ʾ�����˲�ɫͼ��ת��Ϊ�ڰ�ͼ��Ĺ��ܡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetMonochrome
/// \~english
/// \brief Get the status of enabling black and white color conversion.
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns TRUE to enable the conversion of a color image to a mono image.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetMonochrome
MVSDK_API CameraSdkStatus __stdcall CameraGetMonochrome(
	CameraHandle    hCamera,
	BOOL* pbEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���ò�ͼ����ɫ��ת���ܵ�ʹ�ܡ�
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable    TRUE����ʾ����ͼ����ɫ��ת���ܣ����Ի�����ƽ����Ƭ��Ч����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the enable for the color image color flip function.
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable TRUE, means that the image color flip function is enabled, and the effect of similar film negatives can be obtained.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetInverse(
	CameraHandle    hCamera,
	BOOL            bEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���ͼ����ɫ��ת���ܵ�ʹ��״̬��
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable   ָ�룬���ظù���ʹ��״̬�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the status of the image color inversion function.
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns this function enable state.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInverse(
	CameraHandle    hCamera,
	BOOL* pbEnable
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����Զ��ع�ʱ��Ƶ�����ܵ�ʹ��״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable    TRUE��������Ƶ������;FALSE���رոù��ܡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �����ֶ��ع�ģʽ����Ч��
/// \~english
/// \brief Set the anti-strobe function's enable state during auto exposure.
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable TRUE, enable anti-strobe function; FALSE, disable this function.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Not valid for manual exposure mode.
MVSDK_API CameraSdkStatus __stdcall CameraSetAntiFlick(
	CameraHandle    hCamera,
	BOOL            bEnable
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع�ʱ��Ƶ�����ܵ�ʹ��״̬��
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable   ָ�룬���ظù��ܵ�ʹ��״̬��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the anti-strobe function's enable state during auto exposure.
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns the enable state of this function.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAntiFlick(
	CameraHandle    hCamera,
	BOOL* pbEnable
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع�ʱ����Ƶ����Ƶ��ѡ��
/// \param [in] hCamera ����ľ����
/// \param [out] piFrequencySel ָ�룬����ѡ��������š�0:50HZ 1:60HZ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the auto exposure is obtained, the frequency of the deflashing is selected.
/// \param [in] hCamera Camera handle.
/// \param [out] piFrequencySel Returns the selected index number. 0:50HZ 1:60HZ
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLightFrequency(
	CameraHandle    hCamera,
	int* piFrequencySel
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����Զ��ع�ʱ��Ƶ����Ƶ�ʡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iFrequencySel 0:50HZ , 1:60HZ 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the frequency at which the flash disappears during auto exposure.
/// \param [in] hCamera Camera handle.
/// \param [in] iFrequencySel 0:50HZ , 1:60HZ 
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLightFrequency(
	CameraHandle    hCamera,
	int             iFrequencySel
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �趨������ͼ���֡�ʡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iFrameSpeed ѡ���֡��ģʽ�����ţ���Χ��0��tSdkCameraCapbility.iFrameSpeedDesc - 1
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the frame rate of the camera output image.
/// \param [in] hCamera Camera handle.
/// \param [in] iFrameSpeed Frame rate index, ranging from 0 to tSdkCameraCapbility.iFrameSpeedDesc - 1
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetFrameSpeed(
	CameraHandle    hCamera,
	int             iFrameSpeed
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���������ͼ���֡��ѡ�������š�
/// \param [in] hCamera ����ľ����
/// \param [out] piFrameSpeed ����ѡ���֡��ģʽ�����š� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetFrameSpeed
/// \~english
/// \brief Obtain the frame rate selection index number of the camera output image.
/// \param [in] hCamera Camera handle.
/// \param [out] piFrameSpeed Returns the selected frame rate mode index number.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetFrameSpeed
MVSDK_API CameraSdkStatus __stdcall CameraGetFrameSpeed(
	CameraHandle    hCamera,
	int* piFrameSpeed
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �趨�����֡Ƶ(����)����Ƶ(����)�����������������֧�֣�
/// \param [in] hCamera ����ľ����
/// \param [in] RateHZ ֡Ƶ����Ƶ��<=0��ʾ���Ƶ�ʣ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the frame frequency (area) or line frequency (line scan). (only supported by some gige camera)
/// \param [in] hCamera Camera handle.
/// \param [in] RateHZ frame rate or line rate (<=0 means maximum frequency).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetFrameRate(
	CameraHandle    hCamera,
	int             RateHZ
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ�趨�����֡Ƶ(����)����Ƶ(����)
/// \param [in] hCamera ����ľ����
/// \param [out] RateHZ ֡Ƶ����Ƶ��<=0��ʾ���Ƶ�ʣ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the frame frequency (area) or line frequency (line scan).
/// \param [in] hCamera Camera handle.
/// \param [out] RateHZ frame rate or line rate (<=0 means maximum frequency).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetFrameRate(
	CameraHandle    hCamera,
	int* RateHZ
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief �趨������ȡ��Ŀ�����
/// \param [in] hCamera ����ľ����
/// \param [in] iMode  ������ȡ�Ķ��󡣲ο�@link #emSdkParameterMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the target object for parameter access.
/// \param [in] hCamera Camera handle.
/// \param [in] iMode The object accessed by the iMode parameter. Reference @link #emSdkParameterMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetParameterMode(
	CameraHandle    hCamera,
	int             iMode
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ��ȡ������ȡ��Ŀ�����
/// \param [in] hCamera ����ľ����
/// \param [out] piTarget ���ز�����ȡ�Ķ��󡣲ο�@link #emSdkParameterMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gets the target object for the parameter access.
/// \param [in] hCamera Camera handle.
/// \param [out] piTarget Returns the object accessed by the parameter. Reference @link #emSdkParameterMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetParameterMode(
	CameraHandle    hCamera,
	int* piTarget
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ���ò�����ȡ�����롣�������غͱ���ʱ����ݸ���������������ģ��������Ƿ���ػ��߱��档
/// \param [in] hCamera ����ľ����
/// \param [in] uMask     ���롣�ο�@link #emSdkPropSheetMask @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the mask for parameter access. When the parameters are loaded and saved, the mask is used to determine whether each module parameter is loaded or saved.
/// \param [in] hCamera Camera handle.
/// \param [in] uMask mask. Reference @link #emSdkPropSheetMask @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetParameterMask(
	CameraHandle    hCamera,
	UINT            uMask
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ���浱ǰ���������ָ���Ĳ������С�����ṩ��A,B,C,D����ռ������в����ı��档 
/// \param [in] hCamera ����ľ����
/// \param [in] iTeam   �����飬�ο�@link #emSdkParameterTeam @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Save current camera parameters to the specified parameter group. The camera provides A, B, C, D four sets of space for parameter preservation.
/// \param [in] hCamera Camera handle.
/// \param [in] iTeam parameter group, refer to @link #emSdkParameterTeam @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSaveParameter(
	CameraHandle    hCamera,
	int             iTeam
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ���浱ǰ���������ָ�����ļ��С����ļ����Ը��Ƶ���ĵ����Ϲ�����������أ�Ҳ���������������á�
/// \param [in] hCamera ����ľ����
/// \param [in] sFileName �����ļ�������·����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Saves the current camera parameters to the specified file. This file can be copied to another computer for loading by other cameras, or it can be used for parameter backup.
/// \param [in] hCamera Camera handle.
/// \param [in] sFileName Full path to the sFileName parameter file.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSaveParameterToFile(
	CameraHandle  hCamera,
	char* sFileName
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ��PC��ָ���Ĳ����ļ��м��ز������ҹ�˾�������������PC��Ϊ.config��׺���ļ���λ�ڰ�װ�µ�Camera\\Configs�ļ����С�
/// \param [in] hCamera ����ľ����
/// \param [in] sFileName �����ļ�������·����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Load parameters from the parameter file specified on the PC. Our camera parameters are saved on the PC as a .config suffix file, which is located in the Camera\\Configs folder under installation.
/// \param [in] hCamera Camera handle.
/// \param [in] sFileName Full path to the sFileName parameter file.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraReadParameterFromFile(
	CameraHandle    hCamera,
	char* sFileName
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ����ָ����Ĳ���������С�
/// \param [in] hCamera ����ľ����
/// \param [in] iTeam   �����飬�ο�@link #emSdkParameterTeam @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Loads the parameters of the specified group into the camera.
/// \param [in] hCamera Camera handle.
/// \param [in] iTeam parameter group, refer to @link #emSdkParameterTeam @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraLoadParameter(
	CameraHandle    hCamera,
	int             iTeam
);

/// @ingroup API_PARAMETERS
/// \~chinese
/// \brief ��õ�ǰѡ��Ĳ����顣
/// \param [in] hCamera ����ľ����
/// \param [in] piTeam  ָ�룬���ص�ǰѡ��Ĳ����顣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the currently selected parameter group.
/// \param [in] hCamera Camera handle.
/// \param [in] piTeam Returns the currently selected parameter group.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCurrentParameterGroup(
	CameraHandle    hCamera,
	int* piTeam
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �����������ͼ�����ݵķְ���С��
/// \param [in] hCamera ����ľ����
/// \param [in] iPackSel �ְ�����ѡ��������š��ְ����ȿ��ɻ��������Խṹ����@link #tSdkCameraCapbility.pPackLenDesc @endlink��Ա������@link #tSdkCameraCapbility.iPackLenDesc @endlink��Ա���ʾ����ѡ�ķְ�ģʽ������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note Ŀǰ��SDK�汾�У��ýӿڽ���GIGE�ӿ������Ч�������������紫��ķְ���С��
/// \note ����֧�־�֡�����������ǽ���ѡ��8K�ķְ���С��������Ч�Ľ��ʹ�����ռ�õ�CPU����ʱ�䡣
/// \warning �°汾��SDK������ô˺�����SDK���Զ������������Э�����ŵķְ���С
/// \~english
/// \brief Sets the packet size of the camera's transmitted image data.
/// \param [in] hCamera Camera handle.
/// \param [in] iPackSel Index number of the iPackSel packet length selection. The packet length can be expressed by the @link #tSdkCameraCapbility.pPackLenDesc @endlink member in the camera attribute structure. The @link #tSdkCameraCapbility.iPackLenDesc @endlink member represents the maximum number of optional packet modes.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note In the current SDK version, this interface is only valid for GIGE interface cameras and is used to control the packet size of the network transmission.
/// \note For NICs that support Jumbo Frames, we recommend choosing an 8K packet size that can effectively reduce the CPU processing time taken by the transfer.
/// \warning New version of the SDK does not need to call this function, the SDK will automatically negotiate the optimal packet size according to the network conditions
MVSDK_API CameraSdkStatus __stdcall CameraSetTransPackLen(
	CameraHandle    hCamera,
	INT             iPackSel
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������ǰ����ְ���С��ѡ�������š�
/// \param [in] hCamera ����ľ����
/// \param [out] piPackSel  ָ�룬���ص�ǰѡ��ķְ���С�����š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetTransPackLen
/// \~english
/// \brief Gets the selected index number of the camera's current transmission packet size.
/// \param [in] hCamera Camera handle.
/// \param [out] piPackSel Returns the currently selected packet size index number.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetTransPackLen
MVSDK_API CameraSdkStatus __stdcall CameraGetTransPackLen(
	CameraHandle    hCamera,
	INT* piPackSel
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع�ο����ڵ���ʾ״̬��
/// \param [in] hCamera ����ľ����
/// \param [out] pbIsVisible  ָ�룬����TRUE�����ʾ��ǰ���ڻᱻ������ͼ�������ϡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gets the display status of the auto exposure reference window.
/// \param [in] hCamera Camera handle.
/// \param [out] pbIsVisible returns TRUE, indicating that the current window will be overlaid on the image content.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraIsAeWinVisible(
	CameraHandle    hCamera,
	BOOL* pbIsVisible
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����Զ��ع�ο����ڵ���ʾ״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] bIsVisible  TRUE������Ϊ��ʾ��FALSE������ʾ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �����ô���״̬Ϊ��ʾ������@link #CameraImageOverlay @endlink���ܹ�������λ���Ծ��εķ�ʽ������ͼ���ϡ�
/// \~english
/// \brief Sets the display status of the auto exposure reference window.
/// \param [in] hCamera Camera handle.
/// \param [in] bIsVisible TRUE, set to show; FALSE, not show.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note When the window state is set to display, after calling @link #CameraImageOverlay @endlink, the window position can be superimposed on the image in a rectangular manner.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeWinVisible(
	CameraHandle    hCamera,
	BOOL            bIsVisible
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ����Զ��ع�ο����ڵ�λ�á�
/// \param [in] hCamera ����ľ����
/// \param [out] piHOff     ָ�룬���ش���λ�����ϽǺ�����ֵ��
/// \param [out] piVOff     ָ�룬���ش���λ�����Ͻ�������ֵ��
/// \param [out] piWidth    ָ�룬���ش��ڵĿ�ȡ�
/// \param [out] piHeight   ָ�룬���ش��ڵĸ߶ȡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the position of the auto exposure reference window.
/// \param [in] hCamera Camera handle.
/// \param [out] piHOff Returns the abscissa of the upper-left corner of the window.
/// \param [out] piVOff Returns the ordinate value in the upper left corner of the window.
/// \param [out] piWidth Returns the width of the window.
/// \param [out] piHeight Returns the height of the window.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeWindow(
	CameraHandle    hCamera,
	INT* piHOff,
	INT* piVOff,
	INT* piWidth,
	INT* piHeight
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����Զ��ع�Ĳο����ڡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iHOff    �������Ͻǵĺ�����
/// \param [in] iVOff      �������Ͻǵ�������
/// \param [in] iWidth     ���ڵĿ�� 
/// \param [in] iHeight    ���ڵĸ߶�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ���iHOff��iVOff��iWidth��iHeightȫ��Ϊ0���򴰿�����Ϊÿ���ֱ����µľ���1/2��С���������ŷֱ��ʵı仯������仯��
/// \note ���iHOff��iVOff��iWidth��iHeight�������Ĵ���λ�÷�Χ�����˵�ǰ�ֱ��ʷ�Χ�ڣ� ���Զ�ʹ�þ���1/2��С���ڡ�
/// \~english
/// \brief Set the reference window for auto exposure.
/// \param [in] hCamera Camera handle.
/// \param [in] iHOff The horizontal axis of the window in the upper left corner
/// \param [in] iVOff The ordinate of the top left corner of the window
/// \param [in] iWidth width of window
/// \param [in] iHeight Height of window
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note If iHOff, iVOff, iWidth, and iHeight are all 0, the window is set to the center 1/2 size for each resolution. It can follow changes as the resolution changes.
/// \note If the window position range determined by iHOff, iVOff, iWidth, and iHeight exceeds the current resolution range, the centered 1/2 size window is automatically used.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeWindow(
	CameraHandle    hCamera,
	int             iHOff,
	int             iVOff,
	int             iWidth,
	int             iHeight
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief ����ͼ������������������Ϊˮƽ�ʹ�ֱ��������
/// \param [in] hCamera ����ľ����
/// \param [in] iDir     ��ʾ����ķ���0����ʾˮƽ����1����ʾ��ֱ����
/// \param [in] bEnable  TRUE��ʹ�ܾ���;FALSE����ֹ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set image mirroring operation. The mirroring operation is divided into horizontal and vertical directions.
/// \param [in] hCamera Camera handle.
/// \param [in] iDir Indicates the direction of the mirror. 0 means horizontal direction; 1 means vertical direction.
/// \param [in] bEnable TRUE to enable mirroring; FALSE to disable mirroring
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetMirror(
	CameraHandle    hCamera,
	int             iDir,
	BOOL            bEnable
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief ���ͼ��ľ���״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] iDir     ��ʾҪ��õľ�����0����ʾˮƽ����1����ʾ��ֱ����
/// \param [out] pbEnable   ָ�룬����TRUE�����ʾiDir��ָ�ķ�����ʹ�ܡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the mirrored state of the image.
/// \param [in] hCamera Camera handle.
/// \param [in] iDir Indicates the mirroring direction to be obtained. 0 means horizontal direction; 1 means vertical direction.
/// \param [out] pbEnable Returns TRUE, indicating that the direction mirror image of iDir is enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetMirror(
	CameraHandle    hCamera,
	int             iDir,
	BOOL* pbEnable
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief ����Ӳ�����񡣷�Ϊˮƽ�ʹ�ֱ�������򡣣����������ڡ�U3���֧�ִ˹��ܣ�
/// \param [in] hCamera ����ľ����
/// \param [in] iDir     ��ʾ����ķ���0����ʾˮƽ����1����ʾ��ֱ����
/// \param [in] bEnable  TRUE��ʹ�ܾ���;FALSE����ֹ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set up the hardware mirror. Divided into two directions, horizontal and vertical. (Only some GigE and U3 cameras support this feature)
/// \param [in] hCamera Camera handle.
/// \param [in] iDir Indicates the direction of the mirror. 0 means horizontal direction; 1 means vertical direction.
/// \param [in] bEnable TRUE to enable mirroring; FALSE to disable mirroring
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetHardwareMirror(
	CameraHandle    hCamera,
	int             iDir,
	BOOL            bEnable
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief ��ȡ���õ�Ӳ������״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] iDir     ��ʾҪ��õľ�����0����ʾˮƽ����1����ʾ��ֱ����
/// \param [out] pbEnable   ָ�룬����TRUE�����ʾiDir��ָ�ķ�����ʹ�ܡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the hardware mirrored state of the image.
/// \param [in] hCamera Camera handle.
/// \param [in] iDir Indicates the mirroring direction to be obtained. 0 means horizontal direction; 1 means vertical direction.
/// \param [out] pbEnable Returns TRUE, indicating that the direction mirror image of iDir is enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetHardwareMirror(
	CameraHandle    hCamera,
	int             iDir,
	BOOL* pbEnable
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief ����ͼ����ת����
/// \param [in] hCamera ����ľ����
/// \param [in] iRot    ��ʾ��ת�ĽǶȣ���ʱ�뷽�򣩣�0������ת 1:90�� 2:180�� 3:270�ȣ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set image rotation operation
/// \param [in] hCamera Camera handle.
/// \param [in] iRot rotation angle (counterclockwise) (0: no rotation 1:90 degrees 2:180 degrees 3:270 degrees)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetRotate(
	CameraHandle    hCamera,
	int             iRot
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief ���ͼ�����ת״̬��
/// \param [in] hCamera ����ľ����
/// \param [out] iRot     ��ʾҪ��õ���ת���򡣣���ʱ�뷽�򣩣�0������ת 1:90�� 2:180�� 3:270�ȣ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the rotation state of the image.
/// \param [in] hCamera Camera handle.
/// \param [out] iRot Indicates the direction of rotation to get. (Counterclockwise) (0: Do not rotate 1:90 degree 2: 180 degree 3: 270 degree)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetRotate(
	CameraHandle    hCamera,
	int* iRot
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ��ð�ƽ��ο����ڵ�λ�á�
/// \param [in] hCamera ����ľ����
/// \param [out] PiHOff   ָ�룬���زο����ڵ����ϽǺ����� ��
/// \param [out] PiVOff     ָ�룬���زο����ڵ����Ͻ������� ��
/// \param [out] PiWidth    ָ�룬���زο����ڵĿ�ȡ�
/// \param [out] PiHeight   ָ�룬���زο����ڵĸ߶ȡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the position of the white balance reference window.
/// \param [in] hCamera Camera handle.
/// \param [out] PiHOff Returns the top-left abscissa of the reference window.
/// \param [out] PiVOff Returns the upper-left ordinate of the reference window.
/// \param [out] PiWidth Returns the width of the reference window.
/// \param [out] PiHeight Returns the height of the reference window.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetWbWindow(
	CameraHandle    hCamera,
	INT* PiHOff,
	INT* PiVOff,
	INT* PiWidth,
	INT* PiHeight
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ���ð�ƽ��ο����ڵ�λ�á�
/// \param [in] hCamera ����ľ����
/// \param [in] iHOff   �ο����ڵ����ϽǺ����ꡣ
/// \param [in] iVOff     �ο����ڵ����Ͻ������ꡣ
/// \param [in] iWidth    �ο����ڵĿ�ȡ�
/// \param [in] iHeight   �ο����ڵĸ߶ȡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the position of the white balance reference window.
/// \param [in] hCamera Camera handle.
/// \param [in] iHOff The upper left corner of the reference window.
/// \param [in] iVOff The upper left ordinate of the reference window.
/// \param [in] iWidth Width of the reference window.
/// \param [in] iHeight The height of the reference window.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetWbWindow(
	CameraHandle    hCamera,
	INT             iHOff,
	INT             iVOff,
	INT             iWidth,
	INT             iHeight
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ��ð�ƽ�ⴰ�ڵ���ʾ״̬��
/// \param [in] hCamera ����ľ����
/// \param [out] pbShow   ָ�룬����TRUE�����ʾ�����ǿɼ��ġ� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the display status of the white balance window.
/// \param [in] hCamera Camera handle.
/// \param [out] pbShow returns TRUE, indicating that the window is visible.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraIsWbWinVisible(
	CameraHandle    hCamera,
	BOOL* pbShow
);

/// @ingroup API_COLOR
/// \~chinese
/// \brief ���ð�ƽ�ⴰ�ڵ���ʾ״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] bShow      TRUE�����ʾ����Ϊ�ɼ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ڵ���@link #CameraImageOverlay @endlink��ͼ�������Ͻ��Ծ��εķ�ʽ���Ӱ�ƽ��ο����ڵ�λ�á�
/// \~english
/// \brief Sets the display status of the white balance window.
/// \param [in] hCamera Camera handle.
/// \param [in] bShow TRUE indicates that the setting is visible.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note After calling @link #CameraImageOverlay @endlink, the white balance reference window's position will be overlaid on the image content in a rectangular manner.
MVSDK_API CameraSdkStatus __stdcall CameraSetWbWinVisible(
	CameraHandle    hCamera,
	BOOL            bShow
);

/// @ingroup API_ISP
/// \~chinese
/// \brief �������ͼ�������ϵ���ʮ���ߡ���ƽ��ο����ڡ��Զ��ع�ο����ڵ�ͼ�Ρ�ֻ������Ϊ�ɼ�״̬��ʮ���ߺͲο����ڲ��ܱ������ϡ�
/// \param [in] hCamera ����ľ����
/// \param [in] pRgbBuffer ͼ�����ݻ�������
/// \param [in] pFrInfo    ͼ���֡ͷ��Ϣ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The cross-line, white balance reference window, auto exposure reference window, etc. are superimposed on the input image data. Only crosshairs and reference windows that are set to visible can be overlaid.
/// \param [in] hCamera Camera handle.
/// \param [in] pRgbBuffer image data buffer.
/// \param [in] pFrInfo Frame header information for the image.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraImageOverlay(
	CameraHandle    hCamera,
	BYTE* pRgbBuffer,
	tSdkFrameHead* pFrInfo
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ����ָ��ʮ���ߵĲ�����
/// \param [in] hCamera ����ľ����
/// \param [in] iLine    ��ʾҪ���õڼ���ʮ���ߵ�״̬����ΧΪ[0,8]����9����    
/// \param [in] x          ʮ��������λ�õĺ�����ֵ��
/// \param [in] y      ʮ��������λ�õ�������ֵ��
/// \param [in] uColor     ʮ���ߵ���ɫ����ʽΪ(R|(G<<8)|(B<<16))
/// \param [in] bVisible   ʮ���ߵ���ʾ״̬��TRUE����ʾ��ʾ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ֻ������Ϊ��ʾ״̬��ʮ���ߣ��ڵ���@link #CameraImageOverlay @endlink��Żᱻ���ӵ�ͼ���ϡ�
/// \~english
/// \brief Set the parameters for the specified crosshairs.
/// \param [in] hCamera Camera handle.
/// \param [in] iLine Indicates the status of the first few crosshairs. The range is [0,8] for a total of 9.
/// \param [in] x The abscissa of the crosshair center position.
/// \param [in] y The y-axis value of the crosshair center position.
/// \param [in] uColor The color of the crosshair in the format (R|(G<<8)|(B<<16))
/// \param [in] bVisible Crosshair display status. TRUE, indicates the display.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Only crosshairs set to display state will be superimposed on the image after calling @link #CameraImageOverlay @endlink.
MVSDK_API CameraSdkStatus __stdcall CameraSetCrossLine(
	CameraHandle    hCamera,
	int             iLine,
	INT             x,
	INT             y,
	UINT            uColor,
	BOOL            bVisible
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ���ָ��ʮ���ߵ�״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] iLine    ��ʾҪ��ȡ�ĵڼ���ʮ���ߵ�״̬����ΧΪ[0,8]����9����  
/// \param [out] px     ָ�룬���ظ�ʮ��������λ�õĺ����ꡣ
/// \param [out] py     ָ�룬���ظ�ʮ��������λ�õĺ����ꡣ
/// \param [out] pcolor     ָ�룬���ظ�ʮ���ߵ���ɫ����ʽΪ(R|(G<<8)|(B<<16))��
/// \param [out] pbVisible  ָ�룬����TRUE�����ʾ��ʮ���߿ɼ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the status of the designated crosshairs.
/// \param [in] hCamera Camera handle.
/// \param [in] iLine Indicates the status of the first few crosshairs to get. The range is [0,8] for a total of 9.
/// \param [out] px Returns the abscissa of the center of the crosshair.
/// \param [out] py Returns the abscissa of the center of the crosshair.
/// \param [out] pcolor Returns the color of this crosshair in the format (R|(G<<8)|(B<<16)).
/// \param [out] pbVisible returns TRUE, indicating that the crosshairs are visible.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCrossLine(
	CameraHandle    hCamera,
	INT             iLine,
	INT* px,
	INT* py,
	UINT* pcolor,
	BOOL* pbVisible
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief �����������������ṹ�塣�ýṹ���а�������������õĸ��ֲ����ķ�Χ��Ϣ����������غ����Ĳ������أ�Ҳ�����ڶ�̬������������ý��档
/// \param [in] hCamera ����ľ����
/// \param [out] pCameraInfo ָ�룬���ظ�������������Ľṹ�塣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's characteristic description structure. This structure contains range information of various parameters that the camera can set. Determines the return of parameters for related functions and can also be used to dynamically create camera configuration interfaces.
/// \param [in] hCamera Camera handle.
/// \param [out] pCameraInfo Returns the structure of the camera's property description.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCapability(
	CameraHandle            hCamera,
	tSdkCameraCapbility* pCameraInfo
);

/******************************************************/
// ������   : CameraGetCapabilityEx
// �������� : �����������������ṹ�塣�ýṹ���а��������
//        �����õĸ��ֲ����ķ�Χ��Ϣ����������غ����Ĳ���
//        ���أ�Ҳ�����ڶ�̬������������ý��档
// ����     : sDeviceModel    ������ͺţ���ɨ���б��л�ȡ
//            pCameraInfo ָ�룬���ظ�������������Ľṹ�塣
//                        tSdkCameraCapbility��CameraDefine.h�ж��塣
// ����ֵ   : �ɹ�ʱ������CAMERA_STATUS_SUCCESS (0);
//            ���򷵻ط�0ֵ�Ĵ�����,��ο�CameraStatus.h
//            �д�����Ķ��塣
/******************************************************/
MVSDK_API CameraSdkStatus __stdcall CameraGetCapabilityEx(
	char* sDeviceModel,
	tSdkCameraCapbility* pCameraInfo,
	PVOID                   hCameraHandle
);

/// @ingroup API_USERDATA
/// \~chinese
/// \brief ������������кš�
/// \param [in] hCamera ����ľ����
/// \param [in] pbySN    ���кŵĻ������� 
/// \param [in] iLevel   Ҫ�趨�����кż���ֻ����1����2��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ҹ�˾������кŷ�Ϊ3����0�������ҹ�˾�Զ����������кţ�����ʱ�Ѿ��趨�����޷��޸ģ�1����2���������ο���ʹ�á�ÿ�����кų��ȶ���32���ֽڡ�
/// \~english
/// \brief Set the camera's serial number.
/// \param [in] hCamera Camera handle.
/// \param [in] pbySN The buffer for the serial number.
/// \param [in] iLevel The serial number to be set can only be 1 or 2.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Our company camera serial number is divided into 3 levels. Level 0 is our company's custom camera serial number, which has been set at the factory and cannot be modified. Levels 1 and 2 are reserved for secondary development. Each serial number length is 32 bytes.
MVSDK_API CameraSdkStatus __stdcall CameraWriteSN(
	CameraHandle    hCamera,
	BYTE* pbySN,
	INT             iLevel
);

/// @ingroup API_USERDATA
/// \~chinese
/// \brief ��ȡ���ָ����������кš�
/// \param [in] hCamera ����ľ����
/// \param [in] pbySN   ���кŵĻ�������
/// \param [in] iLevel  Ҫ��ȡ�����кż��𡣿���Ϊ0��1��2��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraWriteSN
/// \~english
/// \brief Reads the camera's assigned level serial number.
/// \param [in] hCamera Camera handle.
/// \param [in] pbySN The buffer for the serial number.
/// \param [in] iLevel The sequence number to read. Can be 0, 1 and 2.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraWriteSN
MVSDK_API CameraSdkStatus __stdcall CameraReadSN(
	CameraHandle        hCamera,
	BYTE* pbySN,
	INT                 iLevel
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ����Ӳ������ģʽ�µĴ�����ʱʱ�䣬��λ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [in] uDelayTimeUs Ӳ������ʱ����λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ��Ӳ�����ź����ٺ󣬾���ָ������ʱ���ٿ�ʼ�ɼ�ͼ��
/// \~english
/// \brief Set the trigger delay time in hardware trigger mode, in microseconds.
/// \param [in] hCamera Camera handle.
/// \param [in] uDelayTimeUs Hard trigger delay. Units microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note When the hard trigger signal arrives, after a specified delay, it begins to capture the image.
MVSDK_API CameraSdkStatus __stdcall CameraSetTriggerDelayTime(
	CameraHandle    hCamera,
	UINT            uDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��õ�ǰ�趨��Ӳ������ʱʱ�䡣
/// \param [in] hCamera ����ľ����
/// \param [out] puDelayTimeUs ָ�룬������ʱʱ�䣬��λ΢�롣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the currently set hard trigger delay time.
/// \param [in] hCamera Camera handle.
/// \param [out] puDelayTimeUs Returns the delay time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetTriggerDelayTime(
	CameraHandle    hCamera,
	UINT* puDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ���ô���ģʽ�µĴ���֡���������������Ӳ������ģʽ����Ч��Ĭ��Ϊ1֡����һ�δ����źŲɼ�һ֡ͼ��
/// \param [in] hCamera ����ľ����
/// \param [in] iCount    һ�δ����ɼ���֡����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the number of trigger frames in the trigger mode. Valid for both software and hardware trigger modes. The default is 1 frame, that is, one trigger signal captures a frame of image.
/// \param [in] hCamera Camera handle.
/// \param [in] iCount The number of frames triggered at a time.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetTriggerCount(
	CameraHandle    hCamera,
	INT             iCount
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ���һ�δ�����֡����
/// \param [in] hCamera ����ľ����
/// \param [out] piCount һ�δ����źŲɼ���֡����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the number of trigger frames.
/// \param [in] hCamera Camera handle.
/// \param [out] piCount The number of frames to trigger signal acquisition at one time.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetTriggerCount(
	CameraHandle    hCamera,
	INT* piCount
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ִ��һ��������ִ�к󣬻ᴥ����@link #CameraSetTriggerCount @endlinkָ����֡����
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetTriggerMode
/// \~english
/// \brief Perform a soft trigger. After execution, the number of frames specified by @link #CameraSetTriggerCount @endlink is triggered.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetTriggerMode
MVSDK_API CameraSdkStatus __stdcall CameraSoftTrigger(
	CameraHandle    hCamera
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��������Ĵ���ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] iModeSel   ģʽѡ�������š�0��ʾ�����ɼ�ģʽ��1��ʾ�������ģʽ��2��ʾӲ������ģʽ��  
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera's trigger mode.
/// \param [in] hCamera Camera handle.
/// \param [in] iModeSel mode selects the index number. 0 means continuous acquisition mode; 1 means software trigger mode; 2 means hardware trigger mode.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetTriggerMode(
	CameraHandle    hCamera,
	int             iModeSel
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief �������Ĵ���ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] piModeSel  ָ�룬���ص�ǰѡ����������ģʽ�������š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's trigger mode.
/// \param [in] hCamera Camera handle.
/// \param [out] piModeSel Returns the index of the currently selected camera trigger mode.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetTriggerMode(
	CameraHandle    hCamera,
	INT* piModeSel
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ����IO���Ŷ����ϵ�STROBE�źš����źſ���������ƿ��ƣ�Ҳ�������ⲿ��е���ſ��ơ�
/// \param [in] hCamera ����ľ����
/// \param [in] iMode �����ģʽ���ο�@link #emStrobeControl @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the STROBE signal on the IO pin terminal. This signal can be used for flash control or external mechanical shutter control.
/// \param [in] hCamera Camera handle.
/// \param [in] iMode strobe mode, refer to @link #emStrobeControl @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetStrobeMode(
	CameraHandle    hCamera,
	INT             iMode
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��ȡ��ǰSTROBE�ź����õ�ģʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] piMode ����ģʽ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gets the mode of the current STROBE signal setting.
/// \param [in] hCamera Camera handle.
/// \param [out] piMode Return Mode
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetStrobeMode(
	CameraHandle    hCamera,
	INT* piMode
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��STROBE�źŴ���STROBE_SYNC_WITH_TRIGʱ��ͨ���ú�����������Դ����ź���ʱʱ�䡣
/// \param [in] hCamera ����ľ����
/// \param [in] uDelayTimeUs  ��Դ����źŵ���ʱʱ�䣬��λΪus������Ϊ0��������Ϊ������ 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the STROBE signal is in STROBE_SYNC_WITH_TRIG, set its relative trigger signal delay time by this function.
/// \param [in] hCamera Camera handle.
/// \param [in] uDelayTimeUs Delay time relative to the trigger signal, in units of us. Can be 0, but it cannot be negative.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetStrobeDelayTime(
	CameraHandle    hCamera,
	UINT            uDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��STROBE�źŴ���STROBE_SYNC_WITH_TRIGʱ��ͨ���ú����������Դ����ź���ʱʱ�䡣
/// \param [in] hCamera ����ľ����
/// \param [out] upDelayTimeUs     ָ�룬������ʱʱ�䣬��λus��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the STROBE signal is in STROBE_SYNC_WITH_TRIG, the relative trigger signal delay time is obtained through this function.
/// \param [in] hCamera Camera handle.
/// \param [out] upDelayTimeUs Returns the delay time in us.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetStrobeDelayTime(
	CameraHandle    hCamera,
	UINT* upDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��STROBE�źŴ���STROBE_SYNC_WITH_TRIGʱ��ͨ���ú��������������ȡ�
/// \param [in] hCamera ����ľ����
/// \param [in] uTimeUs ����Ŀ�ȣ���λΪʱ��us��  
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the STROBE signal is in STROBE_SYNC_WITH_TRIG, set its pulse width by this function.
/// \param [in] hCamera Camera handle.
/// \param [in] uTimeUs The width of the pulse in units of time us.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetStrobePulseWidth(
	CameraHandle    hCamera,
	UINT            uTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��STROBE�źŴ���STROBE_SYNC_WITH_TRIGʱ��ͨ���ú�������������ȡ�
/// \param [in] hCamera ����ľ����
/// \param [out] upTimeUs  ָ�룬���������ȡ���λΪus��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the STROBE signal is at STROBE_SYNC_WITH_TRIG, its pulse width is obtained by this function.
/// \param [in] hCamera Camera handle.
/// \param [out] upTimeUs Returns the pulse width. The unit is us.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetStrobePulseWidth(
	CameraHandle    hCamera,
	UINT* upTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��STROBE�źŴ���STROBE_SYNC_WITH_TRIGʱ��ͨ���ú�����������Ч��ƽ�ļ��ԡ�Ĭ��Ϊ����Ч���������źŵ���ʱ��STROBE�źű����ߡ�
/// \param [in] hCamera ����ľ����
/// \param [in] uPolarity STROBE�źŵļ��ԣ�0Ϊ�͵�ƽ��Ч��1Ϊ�ߵ�ƽ��Ч��Ĭ��Ϊ�ߵ�ƽ��Ч��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the STROBE signal is at STROBE_SYNC_WITH_TRIG, the polarity of its active level is set by this function. The default is high active. When the trigger signal arrives, the STROBE signal is pulled high.
/// \param [in] hCamera Camera handle.
/// \param [in] uPolarity Polarity of STROBE signal, 0 is active low and 1 is active high. The default is active high.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetStrobePolarity(
	CameraHandle    hCamera,
	INT             uPolarity
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��������ǰSTROBE�źŵ���Ч���ԡ�Ĭ��Ϊ�ߵ�ƽ��Ч��
/// \param [in] hCamera ����ľ����
/// \param [in] upPolarity    ָ�룬����STROBE�źŵ�ǰ����Ч���ԡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Obtain the effective polarity of the camera's current STROBE signal. The default is active high.
/// \param [in] hCamera Camera handle.
/// \param [in] upPolarity Returns the current effective polarity of the STROBE signal.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetStrobePolarity(
	CameraHandle    hCamera,
	INT* upPolarity
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��������ⴥ���źŵ����ࡣ�ϱ��ء��±��ء����߸ߡ��͵�ƽ��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] iType   �ⴥ���ź����࣬�ο�@link #emExtTrigSignal @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the type of trigger signal outside the camera. Upper edge, lower edge, or high and low level.
/// \param [in] hCamera Camera handle.
/// \param [in] iType External trigger signal type, refer to @link #emExtTrigSignal @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetExtTrigSignalType(
	CameraHandle    hCamera,
	INT             iType
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��������ǰ�ⴥ���źŵ����ࡣ
/// \param [in] hCamera ����ľ����
/// \param [out] ipType ָ�룬�����ⴥ���ź����࣬�ο�@link #emExtTrigSignal @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the type of camera's current external trigger signal.
/// \param [in] hCamera Camera handle.
/// \param [out] ipType Returns the type of external trigger signal, see @link #emExtTrigSignal @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetExtTrigSignalType(
	CameraHandle    hCamera,
	INT* ipType
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief �����ⴥ��ģʽ�£�������ŵķ�ʽ��Ĭ��Ϊ��׼���ŷ�ʽ�����ֹ������ŵ�CMOS���֧��GRR��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] iType   �ⴥ�����ŷ�ʽ���ο�@link #emExtTrigShutterMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief In the external trigger mode, the camera shutter mode defaults to the standard shutter mode. Part of the rolling shutter CMOS camera supports GRR mode.
/// \param [in] hCamera Camera handle.
/// \param [in] iType triggers the shutter. Reference @link #emExtTrigShutterMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetExtTrigShutterType(
	CameraHandle    hCamera,
	INT             iType
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ����ⴥ��ģʽ�£�������ŵķ�ʽ
/// \param [in] hCamera ����ľ����
/// \param [out] ipType    ָ�룬���ص�ǰ�趨���ⴥ�����ŷ�ʽ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetExtTrigShutterType
/// \~english
/// \brief Get the camera shutter mode in external trigger mode
/// \param [in] hCamera Camera handle.
/// \param [out] ipType Returns the currently set external trigger shutter mode.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetExtTrigShutterType
MVSDK_API CameraSdkStatus __stdcall  CameraGetExtTrigShutterType(
	CameraHandle    hCamera,
	INT* ipType
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief �����ⴥ���ź���ʱʱ�䣬Ĭ��Ϊ0����λΪ΢�롣 
/// \param [in] hCamera ����ľ����
/// \param [in] uDelayTimeUs  ��ʱʱ�䣬��λΪ΢�룬Ĭ��Ϊ0.
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the delay time of external trigger signal. The default is 0 and the unit is microsecond.
/// \param [in] hCamera Camera handle.
/// \param [in] uDelayTimeUs Delay time in microseconds. Default is 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetExtTrigDelayTime(
	CameraHandle    hCamera,
	UINT            uDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ������õ��ⴥ���ź���ʱʱ�䣬Ĭ��Ϊ0����λΪ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [out] upDelayTimeUs ������ʱ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the set external trigger signal delay time, the default is 0, the unit is microseconds.
/// \param [in] hCamera Camera handle.
/// \param [out] upDelayTimeUs trigger delay
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetExtTrigDelayTime(
	CameraHandle    hCamera,
	UINT* upDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief �����ⴥ���ź���ʱ����ʱ�䣬Ĭ��Ϊ0����λΪ΢�롣�����õ�ֵuDelayTimeUs��Ϊ0ʱ��������յ��ⴥ���źź󣬽���ʱuDelayTimeUs��΢����ٽ���ͼ�񲶻񡣲��һ����ʱ�ڼ��յ��Ĵ����źŻ�����������������ź�Ҳ����ʱuDelayTimeUs��΢�����Ч����󻺴����128����
/// \param [in] hCamera ����ľ����
/// \param [in] uDelayTimeUs  ��ʱʱ�䣬��λΪ΢�룬Ĭ��Ϊ0.
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the delay activation time of the external trigger signal. The default is 0, and the unit is microsecond. When the set value uDelayTimeUs is not 0, after the camera receives the external trigger signal, it will delay uDelayTimeUs for several microseconds before performing image capture. And the trigger signal received during the delay period will be buffered, and the buffered signal will also take effect after a delay of uDelayTimeUs (the maximum number of buffers is 128).
/// \param [in] hCamera Camera handle.
/// \param [in] uDelayTimeUs Delay time in microseconds. Default is 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetExtTrigBufferedDelayTime(
	CameraHandle    hCamera,
	UINT            uDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ������õ��ⴥ���ź���ʱ����ʱ�䣬Ĭ��Ϊ0����λΪ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [out] puDelayTimeUs ������ʱ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gets the delay activation time of the external trigger signal. The default is 0 and the unit is microsecond.
/// \param [in] hCamera Camera handle.
/// \param [out] puDelayTimeUs trigger delay
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetExtTrigBufferedDelayTime(
	CameraHandle    hCamera,
	UINT* puDelayTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief �����ⴥ���źż��ʱ�䣬Ĭ��Ϊ0����λΪ΢�롣 
/// \param [in] hCamera ����ľ����
/// \param [in] uTimeUs  ���ʱ�䣬��λΪ΢�룬Ĭ��Ϊ0.
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the interval time of external trigger signal. The default is 0 and the unit is microsecond.
/// \param [in] hCamera Camera handle.
/// \param [in] uTimeUs Interval time in microseconds. Default is 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetExtTrigIntervalTime(
	CameraHandle    hCamera,
	UINT            uTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ������õ��ⴥ���źż��ʱ�䣬Ĭ��Ϊ0����λΪ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [out] upTimeUs �������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the set external trigger signal interval time, the default is 0, the unit is microseconds.
/// \param [in] hCamera Camera handle.
/// \param [out] upTimeUs trigger interval
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetExtTrigIntervalTime(
	CameraHandle    hCamera,
	UINT* upTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ��������ⴥ���źŵ�����ʱ�䡣Ĭ��Ϊ0����λΪ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [in] uTimeUs ʱ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the debouncing time of the trigger signal outside the camera. The default is 0 and the unit is microseconds.
/// \param [in] hCamera Camera handle.
/// \param [in] uTimeUs time
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraSetExtTrigJitterTime(
	CameraHandle    hCamera,
	UINT            uTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ������õ�����ⴥ������ʱ�䣬Ĭ��Ϊ0.��λΪ΢�롣
/// \param [in] hCamera ����ľ����
/// \param [out] upTimeUs ʱ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the set camera trigger debounce time, the default is 0. The unit is microseconds.
/// \param [in] hCamera Camera handle.
/// \param [out] upTimeUs time
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetExtTrigJitterTime(
	CameraHandle    hCamera,
	UINT* upTimeUs
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief �������ⴥ������������
/// \param [in] hCamera ����ľ����
/// \param [out] puCapabilityMask  ָ�룬���ظ�����ⴥ���������룬����ο�CameraDefine.h��EXT_TRIG_MASK_ ��ͷ�ĺ궨�塣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the triggering attribute mask outside the camera
/// \param [in] hCamera Camera handle.
/// \param [out] puCapabilityMask Returns the mask of the camera's triggering property, masked by the macro definition at the beginning of EXT_TRIG_MASK_ in CameraDefine.h.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraGetExtTrigCapability(
	CameraHandle    hCamera,
	UINT* puCapabilityMask
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ���ⴥ���ź�Ϊ��ƽģʽʱ����ʱֹͣ���������ֱ����ƽ�ź���������������
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the external trigger signal is in level mode, it temporarily stops triggering the camera until the level signal jumps and continues to trigger.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall  CameraPauseLevelTrigger(
	CameraHandle    hCamera
);

/// @ingroup API_ROI
/// \~chinese
/// \brief ���ץ��ģʽ�µķֱ���ѡ�������š�
/// \param [in] hCamera ����ľ����
/// \param [out] pImageResolution ָ�룬����ץ��ģʽ�ķֱ��ʡ� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the resolution selection index number in snap mode.
/// \param [in] hCamera Camera handle.
/// \param [out] pImageResolution Pointer to return the resolution of the snap mode.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetResolutionForSnap(
	CameraHandle            hCamera,
	tSdkImageResolution* pImageResolution
);

/// @ingroup API_ROI
/// \~chinese
/// \brief ����ץ��ģʽ��������ͼ��ķֱ��ʡ�
/// \param [in] hCamera ����ľ����
/// \param [in] pImageResolution �ֱ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ���pImageResolution->iWidth = pImageResolution->iHeight = 0�����ʾ�趨Ϊ���浱ǰԤ���ֱ��ʡ�ץ�ĵ���ͼ��ķֱ��ʻ�͵�ǰ�趨��Ԥ���ֱ���һ����
/// \~english
/// \brief Sets the resolution of the camera's output image in snap shot mode.
/// \param [in] hCamera Camera handle.
/// \param [in] pImageResolution Resolution
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note If pImageResolution->iWidth = pImageResolution->iHeight = 0, then it is set to follow the current preview resolution. The resolution of the captured image will be the same as the currently set preview resolution.
MVSDK_API CameraSdkStatus __stdcall CameraSetResolutionForSnap(
	CameraHandle            hCamera,
	tSdkImageResolution* pImageResolution
);

/// @ingroup API_ROI
/// \~chinese
/// \brief �򿪷ֱ����Զ�����壬��ͨ�����ӻ��ķ�ʽ������һ���Զ���ֱ��ʡ�
/// \param [in] hCamera ����ľ����
/// \param [out] pImageCustom ָ�룬�����Զ���ķֱ��ʡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Open the resolution custom panel and configure a custom resolution visually.
/// \param [in] hCamera Camera handle.
/// \param [out] pImageCustom Returns the custom resolution.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCustomizeResolution(
	CameraHandle            hCamera,
	tSdkImageResolution* pImageCustom
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief �򿪲ο������Զ�����塣��ͨ�����ӻ��ķ�ʽ�����һ���Զ��崰�ڵ�λ�á�һ�������Զ����ƽ����Զ��ع�Ĳο����ڡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iWinType   Ҫ���ɵĲο����ڵ���;��0:�Զ��ع�ο����ڣ�1:��ƽ��ο����ڡ�
/// \param [in] hParent    ���øú����Ĵ��ڵľ��������ΪNULL��
/// \param [out] piHOff     ָ�룬�����Զ��崰�ڵ����ϽǺ����ꡣ
/// \param [out] piVOff     ָ�룬�����Զ��崰�ڵ����Ͻ������ꡣ
/// \param [out] piWidth    ָ�룬�����Զ��崰�ڵĿ�ȡ� 
/// \param [out] piHeight   ָ�룬�����Զ��崰�ڵĸ߶ȡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Open the reference window custom panel. And through a visual way to get a custom window location. It is generally a reference window with custom white balance and auto exposure.
/// \param [in] hCamera Camera handle.
/// \param [in] iWinType Purpose of the reference window to be generated. 0: Auto exposure reference window; 1: White balance reference window.
/// \param [in] hParent The handle of the window that called the function. Can be NULL.
/// \param [out] piHOff Returns the upper left-hand abscissa of the custom window.
/// \param [out] piVOff Returns the upper left ordinate of the custom window.
/// \param [out] piWidth Returns the width of the custom window.
/// \param [out] piHeight Returns the height of the custom window.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCustomizeReferWin(
	CameraHandle    hCamera,
	INT             iWinType,
	HWND            hParent,
	INT* piHOff,
	INT* piVOff,
	INT* piWidth,
	INT* piHeight
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ��������������ô�����ʾ״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] bShow    TRUE����ʾ;FALSE�����ء�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �����ȵ���@link #CameraCreateSettingPage @endlink�ɹ���������������ô��ں󣬲��ܵ��ñ�����������ʾ��
/// \~english
/// \brief Set the camera property configuration window display status.
/// \param [in] hCamera Camera handle.
/// \param [in] bShow TRUE, show; FALSE, hide.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note You must call @link #CameraCreateSettingPage @endlink successfully to create the camera property configuration window before calling this function to display.
MVSDK_API CameraSdkStatus __stdcall CameraShowSettingPage(
	CameraHandle    hCamera,
	BOOL            bShow
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ������������������ô��ڡ����øú�����SDK�ڲ��������������������ô��ڣ�ʡȥ�������¿���������ý����ʱ�䡣ǿ�ҽ���ʹ����ʹ�øú�����SDKΪ�����������ô��ڡ�
/// \param [in] hCamera ����ľ����
/// \param [in] hParent       Ӧ�ó��������ڵľ��������ΪNULL��
/// \param [in] pWinText      �ַ���ָ�룬������ʾ�ı�������
/// \param [in] pCallbackFunc ������Ϣ�Ļص�����������Ӧ���¼�����ʱ��pCallbackFunc��ָ��ĺ����ᱻ����
/// \param [in] pCallbackCtx  �ص������ĸ��Ӳ���������ΪNULL��
/// \param [in] uReserved     Ԥ������������Ϊ0��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Create the camera's property configuration window. Call this function, the SDK will help you create a camera configuration window, eliminating the need to redevelop the camera configuration interface. It is strongly recommended that you use this function to have the SDK create a configuration window for you.
/// \param [in] hCamera Camera handle.
/// \param [in] hParent Handle to the main window of the application. Can be NULL.
/// \param [in] pWinText string pointer, window title bar.
/// \param [in] pCallbackFunc Callback function of window message. The function pointed to by pCallbackFunc will be called when the corresponding event occurs.
/// \param [in] pCallbackCtx Additional parameters for the callback function. Can be NULL.
/// \param [in] uReserved Reserved. Must be set to 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCreateSettingPage(
	CameraHandle            hCamera,
	HWND                    hParent,
	char* pWinText,
	CAMERA_PAGE_MSG_PROC    pCallbackFunc,
	PVOID                   pCallbackCtx,
	UINT                    uReserved
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ʹ��Ĭ�ϲ���������������������ô��ڡ�
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Use the default parameters to create the camera's property configuration window.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCreateSettingPageEx(
	CameraHandle            hCamera
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ����������ô��ڵļ���ҳ�档������ô����ж����ҳ�湹�ɣ��ú��������趨��ǰ��һ����ҳ��Ϊ����״̬����ʾ����ǰ�ˡ�
/// \param [in] hCamera ����ľ����
/// \param [in] index   ��ҳ��������š��ο�@link #emSdkPropSheetMask @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the camera configuration window's activation page. The camera configuration window is composed of multiple sub-pages. This function can set which sub-page is currently active and displayed at the forefront.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index number of the subpage. Reference @link #emSdkPropSheetMask @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetActiveSettingSubPage(
	CameraHandle    hCamera,
	INT             index
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ���������ҳ����Ϊ�Ӵ��ڷ�񣬲���ָ�����ĸ����ڡ�
/// \param [in] hCamera ����ľ����
/// \param [in] hParentWnd �����ھ����ΪNULL(0)��ָ�����ҳΪ�������ڡ�
/// \param [in] Flags ���ܱ�־λ��bit0: ���ر�������bit1-31: ����(����Ϊ0)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the camera configuration page to child window style and specifies its parent window.
/// \param [in] hCamera Camera handle.
/// \param [in] hParentWnd The parent window handle, NULL (0) restores the configuration page to a popup window.
/// \param [in] Flags function flag, bit0: Hide title bar, bit1-31: Reserved (must be 0)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetSettingPageParent(
	CameraHandle    hCamera,
	HWND            hParentWnd,
	DWORD			Flags
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ��ȡ�������ҳ�Ĵ��ھ����
/// \param [in] hCamera ����ľ����
/// \param [out] hWnd ��������ҳ�Ĵ��ھ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Gets the window handle of the camera configuration page.
/// \param [in] hCamera Camera handle.
/// \param [out] hWnd Returns the window handle of the configuration page.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetSettingPageHWnd(
	CameraHandle    hCamera,
	HWND* hWnd
);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese
/// \brief ˢ���������ҳ
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Refresh camera configuration page
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraUpdateSettingPage(
	CameraHandle    hCamera
);

MVSDK_API CameraSdkStatus __stdcall CameraSpecialControl(
	CameraHandle    hCamera,
	DWORD           dwCtrlCode,
	DWORD           dwParam,
	LPVOID          lpData
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ����������֡�ʵ�ͳ����Ϣ����������֡�Ͷ�֡�������
/// \param [in] hCamera ����ľ����
/// \param [out] psFrameStatistic ָ�룬����ͳ����Ϣ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the frame rate statistics of the camera, including error frame and frame loss.
/// \param [in] hCamera Camera handle.
/// \param [out] psFrameStatistic Returns statistics.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetFrameStatistic(
	CameraHandle            hCamera,
	tSdkFrameStatistic* psFrameStatistic
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ����ͼ����ģ���ʹ��״̬��
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable   TRUE��ʹ�ܣ�FALSE����ֹ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the enable status of the image noise reduction module.
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable TRUE, enable; FALSE, disable.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetNoiseFilter(
	CameraHandle    hCamera,
	BOOL            bEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���ͼ����ģ���ʹ��״̬��
/// \param [in] hCamera ����ľ����
/// \param [out] pEnable   ָ�룬����״̬��TRUE��Ϊʹ�ܡ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the image noise reduction module's enable state.
/// \param [in] hCamera Camera handle.
/// \param [out] pEnable Returns status. TRUE, to enable.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetNoiseFilterState(
	CameraHandle    hCamera,
	BOOL* pEnable
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��λͼ��ɼ���ʱ�������0��ʼ��
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Resets the time stamp of the image acquisition, starting from 0.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraRstTimeStamp(
	CameraHandle    hCamera
);

/// @ingroup API_USERDATA
/// \~chinese
/// \brief ���û��Զ�������ݱ��浽����ķ����Դ洢���С�
/// \param [in] hCamera ����ľ����
/// \param [in] uStartAddr  ��ʼ��ַ����0��ʼ��
/// \param [in] pbData      ���ݻ�����ָ��
/// \param [in] ilen        д�����ݵĳ��ȣ�ilen + uStartAddr����С���û�����󳤶�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ÿ���ͺŵ��������֧�ֵ��û���������󳤶Ȳ�һ�������Դ��豸�����������л�ȡ�ó�����Ϣ��
/// \~english
/// \brief Save user-defined data to the camera's non-volatile memory.
/// \param [in] hCamera Camera handle.
/// \param [in] uStartAddr Start address, starting from 0.
/// \param [in] pbData data buffer pointer
/// \param [in] ilen The length of the write data, ilen + uStartAddr must be less than the maximum length of the user area
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note The maximum length of the user data area that each model of camera may support is different. This length information can be obtained from the device's feature description.
MVSDK_API CameraSdkStatus __stdcall CameraSaveUserData(
	CameraHandle    hCamera,
	UINT            uStartAddr,
	BYTE* pbData,
	int             ilen
);

/// @ingroup API_USERDATA
/// \~chinese
/// \brief ������ķ����Դ洢���ж�ȡ�û��Զ�������ݡ�
/// \param [in] hCamera ����ľ����
/// \param [in] uStartAddr  ��ʼ��ַ����0��ʼ��
/// \param [out] pbData     ���ݻ�����ָ��
/// \param [in] ilen        ���ݵĳ��ȣ�ilen + uStartAddr����С���û�����󳤶�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Read user-defined data from the camera's non-volatile memory.
/// \param [in] hCamera Camera handle.
/// \param [in] uStartAddr Start address, starting from 0.
/// \param [out] pbData data buffer pointer
/// \param [in] ilen The length of the data, ilen + uStartAddr must be less than the maximum length of the user area
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraLoadUserData(
	CameraHandle    hCamera,
	UINT            uStartAddr,
	BYTE* pbData,
	int             ilen
);

/// @ingroup API_USERDATA
/// \~chinese
/// \brief ��ȡ�û��Զ�����豸�ǳơ�
/// \param [in] hCamera ����ľ����
/// \param [out] pName  ָ�룬����ָ��0��β���ַ������豸�ǳƲ�����32���ֽڣ���˸�ָ��ָ��Ļ�����������ڵ���32���ֽڿռ䡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Read user-defined device nicknames.
/// \param [in] hCamera Camera handle.
/// \param [out] pName returns a string that points to the end of 0, the device nickname does not exceed 32 bytes, so the buffer pointed to by this pointer must be greater than or equal to 32 bytes.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetFriendlyName(
	CameraHandle  hCamera,
	char* pName
);

/// @ingroup API_USERDATA
/// \~chinese
/// \brief �����û��Զ�����豸�ǳơ�
/// \param [in] hCamera ����ľ����
/// \param [in] pName   ָ�룬ָ��0��β���ַ������豸�ǳƲ�����32���ֽڣ���˸�ָ��ָ���ַ�������С�ڵ���32���ֽڿռ䡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set user-defined device nicknames.
/// \param [in] hCamera Camera handle.
/// \param [in] pName A string that ends with 0, the device nickname does not exceed 32 bytes, so the pointer to the string must be less than or equal to 32 bytes.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetFriendlyName(
	CameraHandle  hCamera,
	char* pName
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ��ȡSDK�汾��
/// \param [out] pVersionString ָ�룬����SDK�汾�ַ�������ָ��ָ��Ļ�������С�������32���ֽ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Read the SDK version number
/// \param [out] pVersionString Returns the SDK version string. The buffer pointed to by this pointer must be larger than 32 bytes
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSdkGetVersionString(
	char* pVersionString
);

/******************************************************/
// ������   : CameraCheckFwUpdate
// �������� : ���̼��汾���Ƿ���Ҫ������
// ����     : hCamera ����ľ������CameraInit������á�
//            pNeedUpdate ָ�룬���ع̼����״̬��TRUE��ʾ��Ҫ����
// ����ֵ   : �ɹ�ʱ������CAMERA_STATUS_SUCCESS (0);
//            ���򷵻ط�0ֵ�Ĵ�����,��ο�CameraStatus.h
//            �д�����Ķ��塣
/******************************************************/
MVSDK_API CameraSdkStatus __stdcall CameraCheckFwUpdate(
	CameraHandle  hCamera,
	BOOL* pNeedUpdate
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ��ù̼��汾���ַ���
/// \param [in] hCamera ����ľ����
/// \param [out] pVersion ����ָ��һ������32�ֽڵĻ����������ع̼��İ汾�ַ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the firmware version string
/// \param [in] hCamera Camera handle.
/// \param [out] pVersion must point to a buffer larger than 32 bytes and return the firmware version string.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetFirmwareVersion(
	CameraHandle  hCamera,
	char* pVersion
);

// ������CameraGetFirmwareVersion��ͬ��Versionƴд����Ϊ�˼����Ա���
// Same function as CameraGetFirmwareVersion. Version misspelled, reserved for compatibility
MVSDK_API CameraSdkStatus __stdcall CameraGetFirmwareVision(
	CameraHandle  hCamera,
	char* pVersion
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ���ָ���豸��ö����Ϣ
/// \param [in] hCamera ����ľ����
/// \param [out] pCameraInfo ָ�룬�����豸��ö����Ϣ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get enumeration information for the specified device
/// \param [in] hCamera Camera handle.
/// \param [out] pCameraInfo Returns the enumeration information for the device.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetEnumInfo(
	CameraHandle    hCamera,
	tSdkCameraDevInfo* pCameraInfo
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ���ָ���豸�ӿڵİ汾
/// \param [in] hCamera ����ľ����
/// \param [out] pVersion ָ��һ������32�ֽڵĻ����������ؽӿڰ汾�ַ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the version of the specified device interface
/// \param [in] hCamera Camera handle.
/// \param [out] pVersion points to a buffer larger than 32 bytes and returns the interface version string.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInerfaceVersion(
	CameraHandle    hCamera,
	char* pVersion
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ����ָ��IO�ĵ�ƽ״̬��IOΪ�����IO�����Ԥ���ɱ�����IO�ĸ�����@link #tSdkCameraCapbility.iOutputIoCounts @endlink������
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [in] uState Ҫ�趨��״̬(GE��SUA: 0(��)  1(��))
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ѷ�����ʹ��CameraSetIOStateEx�����������ͺ���������״ֵ̬ͳһΪ1�� 0��
/// \~english
/// \brief Set the level state of the specified IO. IO is the output IO. The number of programmable output IOs for the camera is determined by @link #tSdkCameraCapbility.iOutputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [in] uState The state to set(GE��SUA: 0(high)  1(low))
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Obsolete, use CameraSetIOStateEx, which has a unified output state value of 1 high and 0 low for all models of cameras
MVSDK_API CameraSdkStatus __stdcall CameraSetIOState(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT        uState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ����ָ��IO�ĵ�ƽ״̬��IOΪ�����IO�����Ԥ���ɱ�����IO�ĸ�����@link #tSdkCameraCapbility.iOutputIoCounts @endlink������
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [in] uState Ҫ�趨��״̬��1Ϊ�ߣ�0Ϊ�ͣ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the level state of the specified IO. IO is the output IO. The number of programmable output IOs for the camera is determined by @link #tSdkCameraCapbility.iOutputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [in] uState The state to set, 1 is high, 0 is low
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetIOStateEx(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT        uState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡָ��IO�ĵ�ƽ״̬��IOΪ�����IO�����Ԥ���ɱ�����IO�ĸ�����@link #tSdkCameraCapbility.iOutputIoCounts @endlink������
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [out] puState ����IO״̬(GE��SUA: 0(��)  1(��))
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ѷ�����ʹ��CameraGetOutPutIOStateEx�����������ͺ���������״ֵ̬ͳһΪ1�� 0��
/// \~english
/// \brief Read the level state of the specified IO. IO is the output IO. The number of programmable output IOs for the camera is determined by @link #tSdkCameraCapbility.iOutputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] puState return IO state(GE��SUA: 0(high)  1(low))
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Obsolete, use CameraGetOutPutIOStateEx, which has a unified output state value of 1 high and 0 low for all models of cameras
MVSDK_API CameraSdkStatus __stdcall CameraGetOutPutIOState(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT* puState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡָ��IO�ĵ�ƽ״̬��IOΪ�����IO�����Ԥ���ɱ�����IO�ĸ�����@link #tSdkCameraCapbility.iOutputIoCounts @endlink������
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [out] puState ����IO״̬��1Ϊ�ߣ�0Ϊ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Read the level state of the specified IO. IO is the output IO. The number of programmable output IOs for the camera is determined by @link #tSdkCameraCapbility.iOutputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] puState return IO state, 1 is high, 0 is low
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetOutPutIOStateEx(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT* puState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡָ��IO�ĵ�ƽ״̬��IOΪ������IO�����Ԥ���ɱ�����IO�ĸ�����@link #tSdkCameraCapbility.iInputIoCounts @endlink������
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [out] puState ָ�룬����IO״̬(GE��SUA: 0(��)  1(��))
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ѷ�����ʹ��CameraGetIOStateEx�����������ͺ����������״ֵ̬ͳһΪ1�� 0��
/// \~english
/// \brief Read the level state of the specified IO, IO is input type IO, the number of programmable output IOs that the camera reserves is decided by @link #tSdkCameraCapbility.iInputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] puState returns IO state(GE��SUA: 0(high)  1(low))
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Obsolete, use CameraGetIOStateEx, which has a unified input state value of 1 high and 0 low for all models of cameras
MVSDK_API CameraSdkStatus __stdcall CameraGetIOState(
	CameraHandle    hCamera,
	INT         iInputIOIndex,
	UINT* puState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡָ��IO�ĵ�ƽ״̬��IOΪ������IO�����Ԥ���ɱ�����IO�ĸ�����@link #tSdkCameraCapbility.iInputIoCounts @endlink������
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [out] puState ָ�룬����IO״̬,1Ϊ�ߣ�0Ϊ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Read the level state of the specified IO, IO is input type IO, the number of programmable output IOs that the camera reserves is decided by @link #tSdkCameraCapbility.iInputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] puState returns IO state, 1 is high, 0 is low
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetIOStateEx(
	CameraHandle    hCamera,
	INT         iInputIOIndex,
	UINT* puState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��������IO��ģʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [in] iMode IOģʽ,�ο�@link #emCameraGPIOMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the input IO mode
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [in] iMode IO mode, reference @link #emCameraGPIOMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetInPutIOMode(
	CameraHandle    hCamera,
	INT         iInputIOIndex,
	INT			iMode
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ����IO��ģʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piMode IOģʽ,�ο�@link #emCameraGPIOMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the input IO mode
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piMode IO mode, reference @link #emCameraGPIOMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInPutIOMode(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	INT* piMode
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief �������IO��ģʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [in] iMode IOģʽ,�ο�@link #emCameraGPIOMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the output IO mode
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [in] iMode IO mode, reference @link #emCameraGPIOMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetOutPutIOMode(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	INT			iMode
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ���IO��ģʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piMode IOģʽ,�ο�@link #emCameraGPIOMode @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the output IO mode
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piMode IO mode, reference @link #emCameraGPIOMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetOutPutIOMode(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	INT* piMode
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ����IO��ģʽ֧������
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piCapbility IOģʽ֧��λ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the mode support capability of the input IO
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO mode support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInPutIOModeCapbility(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	UINT* piCapbility
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ���IO��ģʽ֧������
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piCapbility IOģʽ֧��λ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the mode support capability of the output IO
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO mode support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetOutPutIOModeCapbility(
	CameraHandle    hCamera,
	INT				iOutputIOIndex,
	UINT* piCapbility
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ����PWM������Ĳ���
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [in] iCycle PWM�����ڣ���λ(us)
/// \param [in] uDuty  ռ�ñȣ�ȡֵ1%~99%
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the PWM output parameters
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [in] iCycle Cycle of PWM, unit (us)
/// \param [in] uDuty Occupancy ratio, 1%~99%
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetOutPutPWM(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT		iCycle,
	UINT		uDuty
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ���ñ�������Ч����
/// \param [in] hCamera ����ľ����
/// \param [in] dir ��Ч����0:����ת����Ч   1��˳ʱ�루A�೬ǰ��B��   2:��ʱ�룩
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the effective direction of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [in] dir Valid direction (0: Both positive and negative are valid    1: Clockwise (A phase leads B)    2: Counterclockwise)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetRotaryEncDir(
	CameraHandle    hCamera,
	INT				dir
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ��������Ч����
/// \param [in] hCamera ����ľ����
/// \param [out] dir ��Ч����0:����ת����Ч   1��˳ʱ�루A�೬ǰ��B��   2:��ʱ�룩
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the effective direction of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [out] dir Valid direction (0: Both positive and negative are valid    1: Clockwise (A phase leads B)    2: Counterclockwise)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetRotaryEncDir(
	CameraHandle    hCamera,
	INT* dir
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ���ñ�����Ƶ��
/// \param [in] hCamera ����ľ����
/// \param [in] mul ��Ƶ
/// \param [in] div ��Ƶ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the frequency of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [in] mul frequency multiplier
/// \param [in] div frequency division
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetRotaryEncFreq(
	CameraHandle hCamera,
	INT			mul,
	INT			div
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ������Ƶ��
/// \param [in] hCamera ����ľ����
/// \param [out] mul ��Ƶ
/// \param [out] div ��Ƶ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the frequency of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [out] mul frequency multiplier
/// \param [out] div frequency division
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetRotaryEncFreq(
	CameraHandle hCamera,
	INT* mul,
	INT* div
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��������IO�ĸ�ʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [in] iFormat IO��ʽ,�ο�@link #emCameraGPIOFormat @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the input IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [in] iFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetInPutIOFormat(
	CameraHandle    hCamera,
	INT         iInputIOIndex,
	INT			iFormat
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ����IO�ĸ�ʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piFormat IO��ʽ,�ο�@link #emCameraGPIOFormat @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the input IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInPutIOFormat(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	INT* piFormat
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief �������IO�ĸ�ʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [in] iFormat IO��ʽ,�ο�@link #emCameraGPIOFormat @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the output IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [in] iFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetOutPutIOFormat(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	INT			iFormat
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ���IO�ĸ�ʽ
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piFormat IO��ʽ,�ο�@link #emCameraGPIOFormat @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the output IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetOutPutIOFormat(
	CameraHandle    hCamera,
	INT				iOutputIOIndex,
	INT* piFormat
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ����IO�ĸ�ʽ֧������
/// \param [in] hCamera ����ľ����
/// \param [in] iInputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piCapbility IO��ʽ֧��λ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the format support capability of the input IO
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO format support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetInPutIOFormatCapbility(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	UINT* piCapbility
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief ��ȡ���IO�ĸ�ʽ֧������
/// \param [in] hCamera ����ľ����
/// \param [in] iOutputIOIndex IO�������ţ���0��ʼ��
/// \param [out] piCapbility IO��ʽ֧��λ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the format support capability of the output IO
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO format support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetOutPutIOFormatCapbility(
	CameraHandle    hCamera,
	INT				iOutputIOIndex,
	UINT* piCapbility
);

// @ingroup API_EXPOSURE
/// \~chinese
/// \brief �����Զ��ع�ʱѡ����㷨����ͬ���㷨�����ڲ�ͬ�ĳ�����
/// \param [in] hCamera ����ľ����
/// \param [in] iIspProcessor   ѡ��ִ�и��㷨�Ķ���,�ο�@link #emSdkIspProcessor @endlink
/// \param [in] iAeAlgorithmSel   Ҫѡ����㷨��š���0��ʼ�����ֵ��@link #tSdkCameraCapbility.iAeAlmSwDesc @endlink��@link #tSdkCameraCapbility.iAeAlmHdDesc @endlink������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The algorithm selected when setting up the automatic exposure, different algorithms are suitable for different scenes.
/// \param [in] hCamera Camera handle.
/// \param [in] iIspProcessor Select the object to execute the algorithm, refer to @link #emSdkIspProcessor @endlink
/// \param [in] iAeAlgorithmSel The algorithm number to select. From 0, the maximum value is determined by @link #tSdkCameraCapbility.iAeAlmSwDesc @endlink and @link #tSdkCameraCapbility.iAeAlmHdDesc @endlink.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAeAlgorithm(
	CameraHandle    hCamera,
	INT             iIspProcessor,
	INT             iAeAlgorithmSel
);

// @ingroup API_EXPOSURE
/// \~chinese
/// \brief ��õ�ǰ�Զ��ع���ѡ����㷨
/// \param [in] hCamera ����ľ����
/// \param [in] iIspProcessor   ѡ��ִ�и��㷨�Ķ���,�ο�@link #emSdkIspProcessor @endlink
/// \param [out] piAlgorithmSel   ���ص�ǰѡ����㷨��š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the current auto exposure selected algorithm
/// \param [in] hCamera Camera handle.
/// \param [in] iIspProcessor Select the object to execute the algorithm, refer to @link #emSdkIspProcessor @endlink
/// \param [out] piAlgorithmSel Returns the currently selected algorithm number.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAeAlgorithm(
	CameraHandle    hCamera,
	INT             iIspProcessor,
	INT* piAlgorithmSel
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ����Bayer����ת��ɫ���㷨��
/// \param [in] hCamera ����ľ����
/// \param [in] iIspProcessor   ѡ��ִ�и��㷨�Ķ��󣬲ο�@link #emSdkIspProcessor @endlink
/// \param [in] iAlgorithmSel     Ҫѡ����㷨��š���0��ʼ�����ֵ��tSdkCameraCapbility.iBayerDecAlmSwDesc��tSdkCameraCapbility.iBayerDecAlmHdDesc������  
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set Bayer data to color algorithm.
/// \param [in] hCamera Camera handle.
/// \param [in] iIspProcessor Select the object to execute the algorithm, refer to @link #emSdkIspProcessor @endlink
/// \param [in] iAlgorithmSel The algorithm number to select. From 0, the maximum value is determined by tSdkCameraCapbility.iBayerDecAlmSwDesc and tSdkCameraCapbility.iBayerDecAlmHdDesc.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetBayerDecAlgorithm(
	CameraHandle    hCamera,
	INT             iIspProcessor,
	INT             iAlgorithmSel
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ���Bayer����ת��ɫ��ѡ����㷨��
/// \param [in] hCamera ����ľ����
/// \param [in] iIspProcessor   ѡ��ִ�и��㷨�Ķ��󣬲ο�@link #emSdkIspProcessor @endlink
/// \param [in] piAlgorithmSel  ���ص�ǰѡ����㷨��š�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the algorithm chosen by Bayer data to color.
/// \param [in] hCamera Camera handle.
/// \param [in] iIspProcessor Select the object to execute the algorithm, refer to @link #emSdkIspProcessor @endlink
/// \param [in] piAlgorithmSel Returns the currently selected algorithm number.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetBayerDecAlgorithm(
	CameraHandle    hCamera,
	INT             iIspProcessor,
	INT* piAlgorithmSel
);

// @ingroup API_ISP
/// \~chinese
/// \brief ����ͼ����Ԫ���㷨ִ�ж�����PC�˻����������ִ���㷨�����������ִ��ʱ���ή��PC�˵�CPUռ���ʡ�
/// \param [in] hCamera ����ľ����
/// \param [in] iIspProcessor �ο�@link #emSdkIspProcessor @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the algorithm execution object of the image processing unit. The algorithm is executed by the PC or the camera. When executed by the camera, the CPU usage of the PC is reduced.
/// \param [in] hCamera Camera handle.
/// \param [in] iIspProcessor Reference @link #emSdkIspProcessor @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetIspProcessor(
	CameraHandle    hCamera,
	INT             iIspProcessor
);

// @ingroup API_ISP
/// \~chinese
/// \brief ���ͼ����Ԫ���㷨ִ�ж���
/// \param [in] hCamera ����ľ����
/// \param [out] piIspProcessor ����ѡ��Ķ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the image processing unit's algorithm execution object.
/// \param [in] hCamera Camera handle.
/// \param [out] piIspProcessor returns the selected object
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetIspProcessor(
	CameraHandle    hCamera,
	INT* piIspProcessor
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ����ͼ��ĺڵ�ƽ��׼��Ĭ��ֵΪ0
/// \param [in] hCamera ����ľ����
/// \param [in] iBlackLevel Ҫ�趨�ĵ�ƽֵ����ΧΪ0��255�� 
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the black level reference of the image. The default value is 0
/// \param [in] hCamera Camera handle.
/// \param [in] iBlackLevel The value to set. The range is 0 to 255.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetBlackLevel(
	CameraHandle    hCamera,
	INT         iBlackLevel
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ���ͼ��ĺڵ�ƽ��׼��Ĭ��ֵΪ0
/// \param [in] hCamera ����ľ����
/// \param [out] piBlackLevel ���ص�ǰ�ĺڵ�ƽֵ����ΧΪ0��255��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the black level of the image, the default value is 0
/// \param [in] hCamera Camera handle.
/// \param [out] piBlackLevel Returns the current black level value. The range is 0 to 255.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetBlackLevel(
	CameraHandle    hCamera,
	INT* piBlackLevel
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ����ͼ��İ׵�ƽ��׼��Ĭ��ֵΪ255
/// \param [in] hCamera ����ľ����
/// \param [in] iWhiteLevel Ҫ�趨�ĵ�ƽֵ����ΧΪ0��255��  
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the white level reference of the image. The default value is 255
/// \param [in] hCamera Camera handle.
/// \param [in] iWhiteLevel The level to be set. The range is 0 to 255.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetWhiteLevel(
	CameraHandle    hCamera,
	INT         iWhiteLevel
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ���ͼ��İ׵�ƽ��׼��Ĭ��ֵΪ255
/// \param [in] hCamera ����ľ����
/// \param [out] piWhiteLevel ���ص�ǰ�İ׵�ƽֵ����ΧΪ0��255��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the white level of the image, the default value is 255
/// \param [in] hCamera Camera handle.
/// \param [out] piWhiteLevel Returns the current white level value. The range is 0 to 255.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetWhiteLevel(
	CameraHandle    hCamera,
	INT* piWhiteLevel
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ����@link #CameraImageProcess @endlink������ͼ����������ʽ��
/// \param [in] hCamera ����ľ����
/// \param [in] uFormat	�����ʽ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ֧�ֵĸ�ʽ��CAMERA_MEDIA_TYPE_MONO8��CAMERA_MEDIA_TYPE_MONO16��CAMERA_MEDIA_TYPE_RGB8��CAMERA_MEDIA_TYPE_RGBA8	��CAMERA_MEDIA_TYPE_BGR8��CAMERA_MEDIA_TYPE_BGRA8
/// \~english
/// \brief Sets the output format of image processing for the @link #CameraImageProcess @endlink function.
/// \param [in] hCamera Camera handle.
/// \param [in] uFormat output format.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Supported formats: CAMERA_MEDIA_TYPE_MONO8, CAMERA_MEDIA_TYPE_MONO16, CAMERA_MEDIA_TYPE_RGB8, CAMERA_MEDIA_TYPE_RGBA8, CAMERA_MEDIA_TYPE_BGR8, CAMERA_MEDIA_TYPE_BGRA8
MVSDK_API CameraSdkStatus __stdcall CameraSetIspOutFormat(
	CameraHandle    hCamera,
	UINT            uFormat
);

/// @ingroup API_ISP
/// \~chinese
/// \brief ��ȡ�����ʽ
/// \param [in] hCamera ����ľ����
/// \param [out] puFormat	���ص�ǰ�����ʽ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSetIspOutFormat
/// \~english
/// \brief Get output format
/// \param [in] hCamera Camera handle.
/// \param [out] puFormat returns the current output format
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSetIspOutFormat
MVSDK_API CameraSdkStatus __stdcall CameraGetIspOutFormat(
	CameraHandle    hCamera,
	UINT* puFormat
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ��ô������Ӧ�������ַ���
/// \param [in] iStatusCode		�����롣(������CameraStatus.h��)
/// \return �ɹ�ʱ�����ش������Ӧ���ַ����׵�ַ;���򷵻�NULL��
/// \~english
/// \brief Get the description string corresponding to the error code
/// \param [in] iStatusCode error code. (Defined in CameraStatus.h)
/// When the return is successful, the first address of the string corresponding to the error code is returned; otherwise it returns NULL.
MVSDK_API char* __stdcall CameraGetErrorString(
	CameraSdkStatus     iStatusCode
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ��ýӿڻ�õ�ͼ���Ǿ���������RGB��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pImageData  ����ͼ�����ݵĻ���������С�����uOutFormatָ���ĸ�ʽ��ƥ�䣬�������ݻ����
/// \param [in] uOutFormat	 �����ʽ 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth     ����ָ�룬����ͼ��Ŀ��
/// \param [out] piHeight    ����ָ�룬����ͼ��ĸ߶�
/// \param [in] wTimes      ץȡͼ��ĳ�ʱʱ�䡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����Ҫ���� @link #CameraReleaseImageBuffer @endlink
/// \~english
/// \brief Get a frame of image data. The image obtained by this interface is the processed RGB format.
/// \param [in] hCamera Camera handle.
/// \param [out] pImageData The buffer to receive the image data, the size must match the format specified by uOutFormat, otherwise the data will overflow
/// \param [in] uOutFormat output format 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth Returns the width of the image
/// \param [out] piHeight Returns the height of the image
/// \param [in] wTimes The time-out time for capturing images.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note does not need to call @link #CameraReleaseImageBuffer @endlink
MVSDK_API CameraSdkStatus __stdcall CameraGetImageBufferEx2(
	CameraHandle    hCamera,
	BYTE* pImageData,
	UINT            uOutFormat,
	int* piWidth,
	int* piHeight,
	UINT            wTimes
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ��ýӿڻ�õ�ͼ���Ǿ���������RGB��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pImageData  ����ͼ�����ݵĻ���������С�����uOutFormatָ���ĸ�ʽ��ƥ�䣬�������ݻ����
/// \param [in] uOutFormat	 �����ʽ 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth     ����ָ�룬����ͼ��Ŀ��
/// \param [out] piHeight    ����ָ�룬����ͼ��ĸ߶�
/// \param [out] puTimeStamp ����ͼ��ʱ��� 
/// \param [in] wTimes      ץȡͼ��ĳ�ʱʱ�䡣
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����Ҫ���� @link #CameraReleaseImageBuffer @endlink
/// \~english
/// \brief Get a frame of image data. The image obtained by this interface is the processed RGB format.
/// \param [in] hCamera Camera handle.
/// \param [out] pImageData The buffer to receive the image data, the size must match the format specified by uOutFormat, otherwise the data will overflow
/// \param [in] uOutFormat output format 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth Returns the width of the image
/// \param [out] piHeight Returns the height of the image
/// \param [out] puTimeStamp returns image timestamp
/// \param [in] wTimes The time-out time for capturing images.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note does not need to call @link #CameraReleaseImageBuffer @endlink
MVSDK_API CameraSdkStatus __stdcall CameraGetImageBufferEx3(
	CameraHandle hCamera,
	BYTE* pImageData,
	UINT uOutFormat,
	int* piWidth,
	int* piHeight,
	UINT* puTimeStamp,
	UINT wTimes
);

/// @ingroup API_BASIC
/// \~chinese
/// \brief ��ø������һЩ���ԡ�
/// \param [in] hCamera ����ľ����
/// \param [out] pMaxWidth	    ���ظ�������ֱ��ʵĿ��
/// \param [out] pMaxHeight      ���ظ�������ֱ��ʵĸ߶� 
/// \param [out] pbColorCamera    ���ظ�����Ƿ��ǲ�ɫ�����1��ʾ��ɫ�����0��ʾ�ڰ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get some of the camera's features.
/// \param [in] hCamera Camera handle.
/// \param [out] pMaxWidth Returns the width of the camera's maximum resolution
/// \param [out] pMaxHeight Returns the height of the camera's maximum resolution
/// \param [out] pbColorCamera Returns whether the camera is a color camera. 1 indicates a color camera, 0 indicates a black and white camera
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCapabilityEx2(
	CameraHandle    hCamera,
	int* pMaxWidth,
	int* pMaxHeight,
	int* pbColorCamera
);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief ���������豸���������ӻָ����ֶ�����
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \warning ���Ĭ��ʹ�����Զ����������Զ�����ģʽ��������ñ�������@see CameraSetAutoConnect
/// \~english
/// \brief Reconnect the device to manually reconnect after the connection is restored
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \warning The camera automatically enables reconnection by default. Do not call this function in auto reconnect mode. @see CameraSetAutoConnect
MVSDK_API CameraSdkStatus __stdcall CameraReConnect(
	CameraHandle    hCamera
);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief �������������״̬�����ڼ������Ƿ����
/// \param [in] hCamera ����ľ����
/// \return ������������CAMERA_STATUS_SUCCESS(0)�������ʾ�ѵ���
/// \~english
/// \brief Test camera connection status to detect if camera is dropped
/// \param [in] hCamera Camera handle.
/// \return The connection normally returns CAMERA_STATUS_SUCCESS(0). Otherwise it is dropped
MVSDK_API CameraSdkStatus __stdcall CameraConnectTest(
	CameraHandle    hCamera
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���������LEDʹ��״̬������LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index       LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [in] enable      ʹ��״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera's LED enable status, without the LED's model, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [in] enable enable state
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLedEnable(
	CameraHandle    hCamera,
	int             index,
	BOOL            enable
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������LEDʹ��״̬������LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index       LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [out] enable      ָ�룬����LEDʹ��״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's LED enable status, without the LED's model, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [out] enable Return LED enable status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLedEnable(
	CameraHandle    hCamera,
	int             index,
	BOOL* enable
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���������LED����״̬������LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index       LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [in] onoff	   LED����״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera's LED switch status, without the LED's model, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [in] onoff LED on/off status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLedOnOff(
	CameraHandle    hCamera,
	int             index,
	BOOL            onoff
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������LED����״̬������LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index      LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [out] onoff	   ����LED����״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's LED switch status, without the LED model, this function returns an error code that does not support.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [out] onoff Returns LED switch status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLedOnOff(
	CameraHandle    hCamera,
	int             index,
	BOOL* onoff
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���������LED����ʱ�䣬����LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index        LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [in] duration		LED����ʱ�䣬��λ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera's LED duration, without the LED model, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [in] duration LED duration in milliseconds
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLedDuration(
	CameraHandle    hCamera,
	int             index,
	UINT            duration
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������LED����ʱ�䣬����LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index        LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [out] duration	 ����LED����ʱ�䣬��λ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's LED duration, without the LED model, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [out] duration Returns the LED duration in milliseconds
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLedDuration(
	CameraHandle    hCamera,
	int             index,
	UINT* duration
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���������LED���ȣ�����LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index      LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [in] uBrightness LED����ֵ����Χ0��255. 0��ʾ�رգ�255������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the camera's LED brightness, without the LED model, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [in] uBrightness LED brightness value, range 0 to 255. 0 means off, 255 brightest.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLedBrightness(
	CameraHandle    hCamera,
	int             index,
	UINT            uBrightness
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������LED���ȣ�����LED���ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] index      LED�Ƶ������ţ���0��ʼ�����ֻ��һ���ɿ������ȵ�LED����ò���Ϊ0 ��
/// \param [out] uBrightness ָ�룬����LED����ֵ����Χ0��255. 0��ʾ�رգ�255������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's LED brightness, without the LED model, this function returns an error code that does not support.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index of the LED, starting from 0. If there is only one LED that can control the brightness, this parameter is 0.
/// \param [out] uBrightness Returns the LED brightness value in the range 0 to 255. 0 means off, 255 is the brightest.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetLedBrightness(
	CameraHandle    hCamera,
	int             index,
	UINT* uBrightness
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ʹ�ܻ��߽�ֹ����Ķ������书�ܣ������ù��ܵ��ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] uEnableMask ����ʹ��״̬���룬��Ӧ�ı���λΪ1��ʾʹ�ܡ�0Ϊ��ֹ��ĿǰSDK֧��4���ɱ༭����index��ΧΪ0��3����bit0 ��bit1��bit2��bit3����4�������ʹ��״̬��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note �ù�����Ҫ����������˽��ɼ������������з֣�ֻ����ָ���Ķ����������ߴ���֡�ʡ���������䵽PC�Ϻ󣬻��Զ�ƴ�ӳ��������棬û�б�����Ĳ��֣����ú�ɫ��䡣
/// \~english
/// \brief Enables or disables the camera's multi-zone transfer function. For models without this function, this function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] uEnableMask Area enable mask. The corresponding bit is 1 to enable. 0 is prohibited. Currently, the SDK supports four editable regions. The index range is 0 to 3, that is, bit0, bit1, bit2, and bit3 control the enabling status of the four regions.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note This function is mainly used to split the entire picture collected on the camera side and only transmit specified multiple areas to increase the transmission frame rate. After multiple areas are transferred to the PC, they will be automatically spliced into an entire frame. Parts that have not been transmitted will be filled with black.
MVSDK_API CameraSdkStatus __stdcall CameraEnableTransferRoi(
	CameraHandle    hCamera,
	UINT            uEnableMask
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �����������Ĳü�����������ˣ�ͼ��Ӵ������ϱ��ɼ��󣬽��ᱻ�ü���ָ�������������ͣ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera		����ľ����
/// \param [in] index		ROI����������ţ���0��ʼ��
/// \param [in] X1			ROI��������Ͻ�X����
/// \param [in] Y1			ROI��������Ͻ�Y����
/// \param [in] X2			ROI��������½�X����
/// \param [in] Y2			ROI��������½�Y����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the clipping area for camera transmission. On the camera side, after the image is captured from the sensor, it will be cropped to the specified area for transmission. This function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index number of the ROI region, starting at 0.
/// \param [in] X1 The X coordinate of the upper left corner of ROI area
/// \param [in] Y1 The Y coordinate of the upper left corner of ROI area
/// \param [in] X2 The X coordinate of the lower right corner of ROI area
/// \param [in] Y2 The Y coordinate of the lower right corner of ROI area
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetTransferRoi(
	CameraHandle    hCamera,
	int             index,
	UINT            X1,
	UINT            Y1,
	UINT            X2,
	UINT            Y2
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ�������Ĳü�����������ˣ�ͼ��Ӵ������ϱ��ɼ��󣬽��ᱻ�ü���ָ�������������ͣ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera		����ľ����
/// \param [in] index		ROI����������ţ���0��ʼ��
/// \param [out] pX1		ROI��������Ͻ�X����
/// \param [out] pY1		ROI��������Ͻ�Y����
/// \param [out] pX2		ROI��������½�X����
/// \param [out] pY2		ROI��������½�Y����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the cropped area of the camera transmission. On the camera side, after the image is captured from the sensor, it will be cropped to the specified area for transmission. This function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] index The index number of the ROI region, starting at 0.
/// \param [out] pX1 Returns the X coordinate of the upper left corner of the ROI area
/// \param [out] pY1 Returns the Y coordinate of the upper left corner of the ROI area
/// \param [out] pX2 Returns the X coordinate of the lower right corner of the ROI area
/// \param [out] pY2 Returns the Y coordinate of the lower right corner of the ROI area
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetTransferRoi(
	CameraHandle    hCamera,
	int             index,
	UINT* pX1,
	UINT* pY1,
	UINT* pX2,
	UINT* pY2
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ����һ�ζ�����ڴ�ռ䡣���ܺ�malloc���ƣ����Ƿ��ص��ڴ�����alignָ�����ֽ�������ġ�
/// \param [in] size	 �ռ�Ĵ�С�� 
/// \param [in] align    ��ַ������ֽ�����
/// \return �ɹ�ʱ�����ط�0ֵ����ʾ�ڴ��׵�ַ��ʧ�ܷ���NULL��
/// \note ������ڴ����ʹ��@link #CameraAlignFree @endlink�ͷ�
/// \~english
/// \brief Apply for an aligned memory space. The function is similar to malloc, but the returned memory is aligned with the number of bytes specified by align.
/// \param [in] size Size of the space.
/// \param [in] align The number of aligned bytes.
/// \return Successful a non-zero value is returned indicating the first address of the memory. Fails to return NULL.
/// \note Memory allocated must be freed using @link #CameraAlignFree @endlink
MVSDK_API BYTE* __stdcall CameraAlignMalloc(
	int             size,
	int             align
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief �ͷ���@link #CameraAlignMalloc @endlink����������ڴ�ռ䡣
/// \param [in] membuffer	�ڴ��ַ
/// \~english
/// \brief Releases the memory space allocated by the @link #CameraAlignMalloc @endlink function.
/// \param [in] membuffer memory address
MVSDK_API void __stdcall CameraAlignFree(
	BYTE* membuffer
);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief ���û�����Զ�������Ĭ��Ϊ���á�
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable	ʹ�������������λTRUEʱ��SDK�ڲ��Զ��������Ƿ���ߣ����ߺ��Լ�������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Enables or disables automatic reconnection. The default is enabled.
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable Enables the camera to reconnect. When TRUE, the SDK automatically detects if the camera is dropped and reconnects itself after disconnection.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetAutoConnect(CameraHandle hCamera, BOOL bEnable);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief ��ȡ�Զ�����ʹ��״̬
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable	   ��������Զ�����ʹ��״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get Automatic Reconnect Enable Status
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns the camera's auto reconnect status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetAutoConnect(CameraHandle hCamera, BOOL* pbEnable);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief �������Զ������Ĵ�����ǰ����@link #CameraSetAutoConnect @endlinkʹ������Զ��������ܡ�Ĭ����ʹ�ܵġ�
/// \param [in] hCamera ����ľ����
/// \param [out] puCounts	   �����Զ������Ĵ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the number of times the camera automatically reconnects, provided @link #CameraSetAutoConnect @endlink enables automatic camera reconnection. The default is enabled.
/// \param [in] hCamera Camera handle.
/// \param [out] puCounts returns the number of automatic reconnections
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetReConnectCounts(CameraHandle hCamera, UINT* puCounts);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���û���õ�֡ץȡģʽ��Ĭ��Ϊ���á��������ܽ�USB2.0���֧�֣�
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable	ʹ�ܵ�֡ץȡģʽ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ÿ���ɹ�ץȡ��һ֡��SDK�������ͣ״̬���Ӷ�����ռ��USB������Ҫ���ڶ�����������յĳ�����
/// \~english
/// \brief Enables or disables single-frame crawl mode, which is disabled by default. (This function is only supported by USB2.0 camera)
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable enables single-frame mode
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Whenever a frame is successfully captured, the SDK enters a pause state, so that it no longer occupies the USB bandwidth. It is mainly used in scenes where multiple cameras take pictures.
MVSDK_API CameraSdkStatus __stdcall CameraSetSingleGrabMode(CameraHandle hCamera, BOOL bEnable);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �������ĵ�֡ץȡʹ��״̬
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable ��������ĵ�֡ץȡģʽʹ��״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the camera's single frame capture enable status
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns the camera's single frame capture mode enable status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetSingleGrabMode(CameraHandle hCamera, BOOL* pbEnable);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������ڵ�֡ץȡģʽʱ��ÿ���ɹ�ץȡ��һ֡��SDK�������ͣ״̬�����ô˺�����ʹSDK�˳���ͣ״̬����ʼץȡ��һ֡
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the camera is in the single frame capture mode, the SDK will enter a pause state after successfully capturing a frame. Calling this function will cause the SDK to exit the pause state and start to grab the next frame.
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraRestartGrab(CameraHandle hCamera);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ͼƬ����������
/// \param [in] hCamera ����ľ����
/// \param [in] iAlgorithSel ʹ�õ������㷨,�ο�@link emEvaluateDefinitionAlgorith @endlink�Ķ���
/// \param [in] pbyIn    ����ͼ�����ݵĻ�������ַ������ΪNULL�� 
/// \param [in] pFrInfo  ����ͼ���֡ͷ��Ϣ
/// \param [out] DefinitionValue ���ص������ȹ�ֵ��Խ��Խ������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Image clarity assessment
/// \param [in] hCamera Camera handle.
/// \param [in] iAlgorithSel The currently used evaluation algorithm, see @link emEvaluateDefinitionAlgorith @endlink
/// \param [in] pbyIn The buffer address of the input image data. Cannot be NULL.
/// \param [in] pFrInfo input image frame header information
/// \param [out] DefinitionValue Returns the sharpness value (greater the clearer)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraEvaluateImageDefinition(
	CameraHandle        hCamera,
	INT					iAlgorithSel,
	BYTE* pbyIn,
	tSdkFrameHead* pFrInfo,
	double* DefinitionValue
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief �������ͼ�������л�������
/// \param [inout] pRgbBuffer ͼ�����ݻ�����
/// \param [in] pFrInfo ͼ���֡ͷ��Ϣ
/// \param [in] pFontFileName �����ļ���
/// \param [in] FontWidth ������
/// \param [in] FontHeight ����߶�
/// \param [in] pText Ҫ���������
/// \param [in] Left ���ֵ��������
/// \param [in] Top ���ֵ��������
/// \param [in] Width ���ֵ��������
/// \param [in] Height ���ֵ��������
/// \param [in] TextColor ������ɫRGB
/// \param [in] uFlags �����־,���@link #emCameraDrawTextFlags @endlink�еĶ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Draw text in the input image data
/// \param [inout] pRgbBuffer image data buffer
/// \param [in] pFrInfo frame header information
/// \param [in] pFontFileName font file name
/// \param [in] FontWidth font width
/// \param [in] FontHeight font height
/// \param [in] pText Text to output
/// \param [in] Left text output rectangle
/// \param [in] Top text output rectangle
/// \param [in] Width Output rectangle of text
/// \param [in] Height the output rectangle of the text
/// \param [in] TextColor Text Color RGB
/// \param [in] uFlags output flags, as defined in @link #emCameraDrawTextFlags @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraDrawText(
	BYTE* pRgbBuffer,
	tSdkFrameHead* pFrInfo,
	char const* pFontFileName,
	UINT			FontWidth,
	UINT			FontHeight,
	char const* pText,
	INT				Left,
	INT				Top,
	UINT			Width,
	UINT			Height,
	UINT			TextColor,
	UINT			uFlags
);

/// @ingroup API_ENUM
/// \~chinese
/// \brief ��ָ��IPö��GIGE�豸���������豸�б�����������͵��Բ���ͬһ���ε������
/// \param [in] ppIpList Ŀ��IP
/// \param [in] numIp Ŀ��IP����
/// \param [out] pCameraList �豸�б�����ָ��
/// \param [inout] piNums �豸�ĸ���ָ�룬����ʱ����pCameraList�����Ԫ�ظ�������������ʱ������ʵ���ҵ����豸����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ���
/// \warning piNumsָ���ֵ�����ʼ�����Ҳ�����pCameraList����Ԫ�ظ����������п�������ڴ����
/// \note ���ص������Ϣ�б������acFriendlyName����ġ�������Խ���������ֱ��Ϊ��Camera1���͡�Camera2�������ֺ�����Ϊ��Camera1�����������ǰ�棬��Ϊ��Camera2��������ź��档
/// \~english
/// \brief Enumerates GIGE devices from the specified IP and builds a device list (applicable when the camera and computer are not on the same network segment)
/// \param [in] ppIpList target IP
/// \param [in] numIp number of target IPs
/// \param [out] pCameraList Device list array pointer
/// \param [inout] piNums The number of pointers to the device, the number of elements passed to the pCameraList array at the time of the call. When the function returns, the number of devices actually found is saved.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \warning piNums The value pointed to must be initialized and does not exceed the number of pCameraList array elements, otherwise it may cause memory overflow
/// \note The list of returned camera information will be sorted according to acFriendlyName. For example, after changing the two cameras to the names of "Camera1" and "Camera2," the camera named "Camera1" will be in front, and the camera named "Camera2" will be behind the row.
MVSDK_API CameraSdkStatus __stdcall CameraGigeEnumerateDevice(
	char const** ppIpList,
	int                 numIp,
	tSdkCameraDevInfo* pCameraList,
	int* piNums
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ��ȡGIGE�����IP��ַ
/// \param [in] pCameraInfo ������豸������Ϣ������@link #CameraEnumerateDevice @endlink������á� 
/// \param [out] CamIp ���IP(ע�⣺���뱣֤����Ļ��������ڵ���16�ֽ�)
/// \param [out] CamMask �����������(ע�⣺���뱣֤����Ļ��������ڵ���16�ֽ�)
/// \param [out] CamGateWay �������(ע�⣺���뱣֤����Ļ��������ڵ���16�ֽ�)
/// \param [out] EtIp ����IP(ע�⣺���뱣֤����Ļ��������ڵ���16�ֽ�)
/// \param [out] EtMask ������������(ע�⣺���뱣֤����Ļ��������ڵ���16�ֽ�)
/// \param [out] EtGateWay ��������(ע�⣺���뱣֤����Ļ��������ڵ���16�ֽ�)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the GIGE camera's IP address
/// \param [in] pCameraInfo camera's device description information can be obtained by @link #CameraEnumerateDevice @endlink function.
/// \param [out] CamIp camera IP (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] CamMask camera subnet mask (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] CamGateWay camera gateway (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] EtIp network card IP (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] EtMask subnet mask (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] EtGateWay NIC Gateway (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGigeGetIp(
	tSdkCameraDevInfo* pCameraInfo,
	char* CamIp,
	char* CamMask,
	char* CamGateWay,
	char* EtIp,
	char* EtMask,
	char* EtGateWay
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ����GIGE�����IP��ַ
/// \param [in] pCameraInfo ������豸������Ϣ������@link #CameraEnumerateDevice @endlink������á� 
/// \param [in] Ip ���IP(�磺192.168.1.100)
/// \param [in] SubMask �����������(�磺255.255.255.0)
/// \param [in] GateWay �������(�磺192.168.1.1)
/// \param [in] bPersistent TRUE: �������Ϊ�̶�IP��FALSE����������Զ�����IP�����Բ���Ip, SubMask, GateWay��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the GIGE camera's IP address
/// \param [in] pCameraInfo camera's device description information can be obtained by @link #CameraEnumerateDevice @endlink function.
/// \param [in] Ip camera IP (eg 192.168.1.100)
/// \param [in] SubMask camera subnet mask (eg 255.255.255.0)
/// \param [in] GateWay Camera Gateway (eg 192.168.1.1)
/// \param [in] bPersistent TRUE: Set camera to fixed IP, FALSE: Set camera to assign IP automatically (ignoring parameters Ip, SubMask, GateWay)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGigeSetIp(
	tSdkCameraDevInfo* pCameraInfo,
	char const* Ip,
	char const* SubMask,
	char const* GateWay,
	BOOL bPersistent
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ��ȡGIGE�����MAC��ַ
/// \param [in] pCameraInfo ������豸������Ϣ������@link #CameraEnumerateDevice @endlink������á� 
/// \param [out] CamMac ���MAC(ע�⣺���뱣֤����Ļ��������ڵ���18�ֽ�)
/// \param [out] EtMac ����MAC(ע�⣺���뱣֤����Ļ��������ڵ���18�ֽ�)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Select the LUT table in the preset LUT mode.
/// \param [in] pCameraInfo camera's device description information can be obtained by @link #CameraEnumerateDevice @endlink function.
/// \param [out] CamMac camera MAC (Note: must ensure that the incoming buffer is greater than or equal to 18 bytes)
/// \param [out] EtMac network card MAC (Note: must ensure that the incoming buffer is greater than or equal to 18 bytes)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGigeGetMac(
	tSdkCameraDevInfo* pCameraInfo,
	char* CamMac,
	char* EtMac
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ʹ�ܿ�����Ӧ
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Enable quick response
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraEnableFastResponse(
	CameraHandle hCamera
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ʹ�ܻ�������
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable TRUE: ʹ�ܻ�������   FALSE: �رջ�������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Enable dead pixel correction
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable TRUE: Enable dead pixel correction FALSE: Turn off dead pixel correction
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetCorrectDeadPixel(
	CameraHandle hCamera,
	BOOL bEnable
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ��ȡ��������ʹ��״̬
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable ����ʹ��״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get dead pixel correction enabled
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns enable state
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetCorrectDeadPixel(
	CameraHandle hCamera,
	BOOL* pbEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ʹ��ƽ��У��
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable     TRUE: ʹ��ƽ��У��   FALSE: �ر�ƽ��У��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Enable flat field correction
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable TRUE: Enable flat field correction FALSE: Turn off flat field correction
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlatFieldingCorrectSetEnable(
	CameraHandle hCamera,
	BOOL bEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��ȡƽ��У��ʹ��״̬
/// \param [in] hCamera ����ľ����
/// \param [out] pbEnable ����ʹ��״̬
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get Plane Correction Enable Status
/// \param [in] hCamera Camera handle.
/// \param [out] pbEnable Returns enable state
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlatFieldingCorrectGetEnable(
	CameraHandle hCamera,
	BOOL* pbEnable
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ����ƽ��У������
/// \param [in] hCamera ����ľ����
/// \param [in] pDarkFieldingImage ����ͼƬ
/// \param [in] pDarkFieldingFrInfo ����ͼƬ��Ϣ
/// \param [in] pLightFieldingImage ����ͼƬ
/// \param [in] pLightFieldingFrInfo ����ͼƬ��Ϣ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set flat field correction parameters
/// \param [in] hCamera Camera handle.
/// \param [in] pDarkFieldingImage dark field image
/// \param [in] pDarkFieldingFrInfo dark field image information
/// \param [in] pLightFieldingImage Brightfield image
/// \param [in] pLightFieldingFrInfo Brightfield image information
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlatFieldingCorrectSetParameter(
	CameraHandle hCamera,
	BYTE const* pDarkFieldingImage,
	tSdkFrameHead const* pDarkFieldingFrInfo,
	BYTE const* pLightFieldingImage,
	tSdkFrameHead const* pLightFieldingFrInfo
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��ȡƽ��У��������״̬
/// \param [in] hCamera ����ľ����
/// \param [out] pbValid ���ز����Ƿ���Ч
/// \param [out] pFilePath ���ز����ļ���·��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get status of flat field correction parameters
/// \param [in] hCamera Camera handle.
/// \param [out] pbValid Return whether the parameter is valid
/// \param [out] pFilePath Returns the path of the parameter file
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlatFieldingCorrectGetParameterState(
	CameraHandle hCamera,
	BOOL* pbValid,
	char* pFilePath
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ����ƽ��У���������ļ�
/// \param [in] hCamera ����ľ����
/// \param [in] pszFileName �ļ�·��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Save flat correction parameters to file
/// \param [in] hCamera Camera handle.
/// \param [in] pszFileName file path
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlatFieldingCorrectSaveParameterToFile(
	CameraHandle hCamera,
	char const* pszFileName
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ���ļ��м���ƽ��У������
/// \param [in] hCamera ����ľ����
/// \param [in] pszFileName �ļ�·��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Load flat field correction parameters from file
/// \param [in] hCamera Camera handle.
/// \param [in] pszFileName file path
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlatFieldingCorrectLoadParameterFromFile(
	CameraHandle hCamera,
	char const* pszFileName
);

/******************************************************/
// ������   : CameraCommonCall
// �������� : �����һЩ���⹦�ܵ��ã����ο���ʱһ�㲻��Ҫ���á�
// ����     : hCamera   ����ľ������CameraInit������á�
//            pszCall   ���ܼ�����
//            pszResult ���ý������ͬ��pszCallʱ�����岻ͬ��
//            uResultBufSize pszResultָ��Ļ��������ֽڴ�С
// ����ֵ   : �ɹ�ʱ������CAMERA_STATUS_SUCCESS (0);
//            ���򷵻ط�0ֵ�Ĵ�����,��ο�CameraStatus.h
//            �д�����Ķ��塣
/******************************************************/
MVSDK_API CameraSdkStatus __stdcall CameraCommonCall(
	CameraHandle    hCamera,
	char const* pszCall,
	char* pszResult,
	UINT			uResultBufSize
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ����3D�������
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable  ���û����
/// \param [in] nCount   ʹ�ü���ͼƬ���н���(2-8��)
/// \param [in] Weights  ����Ȩ�أ��統ʹ��3��ͼƬ���н���������������Դ���3������(0.3,0.3,0.4)�����һ��ͼƬ��Ȩ�ش���ǰ2�š��������Ҫʹ��Ȩ�أ���������������0����ʾ����ͼƬ��Ȩ����ͬ(0.33,0.33,0.33)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set 3D noise reduction parameters
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable enable or disable
/// \param [in] nCount Noise reduction using several pictures (2-8)
/// \param [in] Weights Noise reduction weight, such as when using 3 pictures for noise reduction, this parameter can be passed in 3 floating points (0.3, 0.3, 0.4). The weight of the last picture is larger than the first 2 pictures. . If you do not need to use weights, then pass this parameter to 0, indicating that all images have the same weight (0.33, 0.33, 0.33)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetDenoise3DParams(
	CameraHandle    hCamera,
	BOOL			bEnable,
	int				nCount,
	float* Weights
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��ȡ��ǰ��3D�������
/// \param [in] hCamera ����ľ����
/// \param [out] bEnable  ���û����
/// \param [out] nCount   ʹ���˼���ͼƬ���н���
/// \param [out] bUseWeight �Ƿ�ʹ���˽���Ȩ��
/// \param [out] Weights  ����Ȩ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get current 3D noise reduction parameters
/// \param [in] hCamera Camera handle.
/// \param [out] bEnable enable or disable
/// \param [out] nCount uses several pictures for noise reduction
/// \param [out] bUseWeight whether to use noise reduction weights
/// \param [out] Weights Noise Reduction Weights
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetDenoise3DParams(
	CameraHandle    hCamera,
	BOOL* bEnable,
	int* nCount,
	BOOL* bUseWeight,
	float* Weights
);

/// @ingroup API_ENHANCE
/// \~chinese
/// \brief ��һ��֡����һ�ν��봦��
/// \param [in] InFramesHead  ����֡ͷ
/// \param [in] InFramesData  ����֡����
/// \param [in] nCount   ����֡������
/// \param [in] Weights  ����Ȩ��
/// \param [out] OutFrameHead ���֡ͷ
/// \param [out] OutFrameData ���֡����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Perform a noise reduction on a group of frames
/// \param [in] InFramesHead input frame header
/// \param [in] InFramesData input frame data
/// \param [in] nCount Number of input frames
/// \param [in] Weights Noise Reduction Weight
/// \param [out] OutFrameHead output frame header
/// \param [out] OutFrameData output frame data
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraManualDenoise3D(
	tSdkFrameHead* InFramesHead,
	BYTE** InFramesData,
	int				nCount,
	float* Weights,
	tSdkFrameHead* OutFrameHead,
	BYTE* OutFrameData
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief �򿪻���༭���
/// \param [in] hCamera ����ľ����
/// \param [in] hParent    ���øú����Ĵ��ڵľ��������ΪNULL��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Open the dead pixels editing panel
/// \param [in] hCamera Camera handle.
/// \param [in] hParent The handle of the window that called the function. Can be NULL.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCustomizeDeadPixels(
	CameraHandle	hCamera,
	HWND			hParent
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ��ȡ�������
/// \param [in] hCamera ����ľ����
/// \param [out] pRows ����y����
/// \param [out] pCols ����x����
/// \param [out] pNumPixel ����ʱ��ʾ���л������Ĵ�С������ʱ��ʾ���л������з��صĻ���������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ��pRows����pColsΪNULLʱ������������ǰ�Ļ������ͨ��pNumPixel����
/// \~english
/// \brief Reading camera dead pixels
/// \param [in] hCamera Camera handle.
/// \param [out] pRows dead pixels y coordinates
/// \param [out] pCols bad x coordinate
/// \param [out] pNumPixel Inputs the size of the row and column buffers. When returned, it indicates the number of bad pixels returned in the row and column buffers.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note When pRows or pCols is NULL, the function will return the camera's current number of dead pixels through pNumPixel.
MVSDK_API CameraSdkStatus __stdcall CameraReadDeadPixels(
	CameraHandle    hCamera,
	USHORT* pRows,
	USHORT* pCols,
	UINT* pNumPixel
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ����������
/// \param [in] hCamera ����ľ����
/// \param [in] pRows ����y����
/// \param [in] pCols ����x����
/// \param [in] NumPixel ���л������еĻ������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Add camera dead pixels
/// \param [in] hCamera Camera handle.
/// \param [in] pRows dead point y coordinates
/// \param [in] pCols bad x coordinate
/// \param [in] NumPixel Number of dead pixels in row buffer
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraAddDeadPixels(
	CameraHandle    hCamera,
	USHORT* pRows,
	USHORT* pCols,
	UINT			NumPixel
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ɾ�����ָ������
/// \param [in] hCamera ����ľ����
/// \param [in] pRows ����y����
/// \param [in] pCols ����x����
/// \param [in] NumPixel ���л������еĻ������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Delete camera specified dead pixels
/// \param [in] hCamera Camera handle.
/// \param [in] pRows dead point y coordinates
/// \param [in] pCols bad x coordinate
/// \param [in] NumPixel Number of dead pixels in row buffer
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraRemoveDeadPixels(
	CameraHandle    hCamera,
	USHORT* pRows,
	USHORT* pCols,
	UINT			NumPixel
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ɾ����������л���
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Remove all camera's dead pixels
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraRemoveAllDeadPixels(
	CameraHandle    hCamera
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ����������㵽����洢��
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Save camera dead pixels to camera memory
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSaveDeadPixels(
	CameraHandle    hCamera
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ����������㵽�ļ���
/// \param [in] hCamera ����ľ����
/// \param [in] sFileName  �ļ�������·����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Save Camera Dead Points to File
/// \param [in] hCamera Camera handle.
/// \param [in] sFileName Full path to the file.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSaveDeadPixelsToFile(
	CameraHandle    hCamera,
	char const* sFileName
);

/// @ingroup API_DEAD_PIXEL
/// \~chinese
/// \brief ���ļ������������
/// \param [in] hCamera ����ľ����
/// \param [in] sFileName  �ļ�������·����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Loading camera dead pixels from file
/// \param [in] hCamera Camera handle.
/// \param [in] sFileName Full path to the file.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraLoadDeadPixelsFromFile(
	CameraHandle    hCamera,
	char const* sFileName
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ�
/// \param [in] hCamera ����ľ����
/// \param [out] pFrameInfo  ͼ���֡ͷ��Ϣָ�롣
/// \param [out] pbyBuffer   ָ��ͼ������ݵĻ�����ָ�롣
/// \param [in] wTimes ץȡͼ��ĳ�ʱʱ�䡣
/// \param [in] Priority ȡͼ���ȼ� �����@link #emCameraGetImagePriority @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ���˶�һ�����ȼ���������@link #CameraGetImageBuffer @endlink��ͬ
/// \~english
/// \brief Get a frame of image data.
/// \param [in] hCamera Camera handle.
/// \param [out] pFrameInfo Frame header information pointer
/// \param [out] pbyBuffer Pointer to the buffer of data for the image.
/// \param [in] wTimes The time-out time for capturing images.
/// \param [in] Priority map priority See: @link #emCameraGetImagePriority @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Same as @link #CameraGetImageBuffer @endlink except one more priority parameter
MVSDK_API CameraSdkStatus __stdcall CameraGetImageBufferPriority(
	CameraHandle        hCamera,
	tSdkFrameHead* pFrameInfo,
	BYTE** pbyBuffer,
	UINT                wTimes,
	UINT				Priority
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ��ýӿڻ�õ�ͼ���Ǿ���������RGB��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] piWidth    ����ָ�룬����ͼ��Ŀ��
/// \param [out] piHeight   ����ָ�룬����ͼ��ĸ߶�
/// \param [in] wTimes ץȡͼ��ĳ�ʱʱ�䡣��λ���롣
/// \param [in] Priority ȡͼ���ȼ� �����@link #emCameraGetImagePriority @endlink
/// \return �ɹ�ʱ������RGB���ݻ��������׵�ַ;���򷵻�0��
/// \note ���˶�һ�����ȼ���������@link #CameraGetImageBufferEx @endlink��ͬ
/// \~english
/// \brief Get a frame of image data. The image obtained by this interface is the processed RGB format.
/// \param [in] hCamera Camera handle.
/// \param [out] piWidth Returns the width of the image
/// \param [out] piHeight Returns the height of the image
/// \param [in] wTimes The time-out time for capturing images. The unit is milliseconds.
/// \param [in] Priority map priority See: @link #emCameraGetImagePriority @endlink
/// \return Returns the first address of the RGB data buffer when successful; otherwise returns 0.
/// \note Same as @link #CameraGetImageBufferEx @endlink except one more priority parameter
MVSDK_API unsigned char* __stdcall CameraGetImageBufferPriorityEx(
	CameraHandle        hCamera,
	INT* piWidth,
	INT* piHeight,
	UINT                wTimes,
	UINT				Priority
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ��ýӿڻ�õ�ͼ���Ǿ���������RGB��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pImageData  ����ͼ�����ݵĻ���������С�����uOutFormatָ���ĸ�ʽ��ƥ�䣬�������ݻ����
/// \param [in] uOutFormat	 �����ʽ 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth     ����ָ�룬����ͼ��Ŀ��
/// \param [out] piHeight    ����ָ�룬����ͼ��ĸ߶�
/// \param [in] wTimes      ץȡͼ��ĳ�ʱʱ�䡣��λ���롣
/// \param [in] Priority ȡͼ���ȼ� �����@link #emCameraGetImagePriority @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ���˶�һ�����ȼ���������@link #CameraGetImageBufferEx2 @endlink��ͬ
/// \~english
/// \brief Get a frame of image data. The image obtained by this interface is the processed RGB format.
/// \param [in] hCamera Camera handle.
/// \param [out] pImageData The buffer to receive the image data, the size must match the format specified by uOutFormat, otherwise the data will overflow
/// \param [in] uOutFormat Output Format 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth Returns the width of the image
/// \param [out] piHeight Returns the height of the image
/// \param [in] wTimes The time-out time for capturing images. The unit is milliseconds.
/// \param [in] Priority map priority See: @link #emCameraGetImagePriority @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Same as @link #CameraGetImageBufferEx2 @endlink except one more priority parameter
MVSDK_API CameraSdkStatus __stdcall CameraGetImageBufferPriorityEx2(
	CameraHandle    hCamera,
	BYTE* pImageData,
	UINT            uOutFormat,
	int* piWidth,
	int* piHeight,
	UINT            wTimes,
	UINT			Priority
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���һ֡ͼ�����ݡ��ýӿڻ�õ�ͼ���Ǿ���������RGB��ʽ��
/// \param [in] hCamera ����ľ����
/// \param [out] pImageData  ����ͼ�����ݵĻ���������С�����uOutFormatָ���ĸ�ʽ��ƥ�䣬�������ݻ����
/// \param [in] uOutFormat	 �����ʽ 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth     ����ָ�룬����ͼ��Ŀ��
/// \param [out] piHeight    ����ָ�룬����ͼ��ĸ߶�
/// \param [out] puTimeStamp ����ͼ��ʱ��� 
/// \param [in] wTimes      ץȡͼ��ĳ�ʱʱ�䡣
/// \param [in] Priority ȡͼ���ȼ� �����@link #emCameraGetImagePriority @endlink
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ���˶�һ�����ȼ���������@link #CameraGetImageBufferEx3 @endlink��ͬ
/// \~english
/// \brief Get a frame of image data. The image obtained by this interface is the processed RGB format.
/// \param [in] hCamera Camera handle.
/// \param [out] pImageData The buffer to receive the image data, the size must match the format specified by uOutFormat, otherwise the data will overflow
/// \param [in] uOutFormat output format 0:Mono8 1:rgb24 2:rgba32 3:bgr24 4:bgra32
/// \param [out] piWidth Returns the width of the image
/// \param [out] piHeight Returns the height of the image
/// \param [out] puTimeStamp returns image timestamp
/// \param [in] wTimes The time-out time for capturing images.
/// \param [in] Priority map priority See: @link #emCameraGetImagePriority @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Same as @link #CameraGetImageBufferEx3 @endlink except one more priority parameter
MVSDK_API CameraSdkStatus __stdcall CameraGetImageBufferPriorityEx3(
	CameraHandle hCamera,
	BYTE* pImageData,
	UINT uOutFormat,
	int* piWidth,
	int* piHeight,
	UINT* puTimeStamp,
	UINT wTimes,
	UINT Priority
);

/// @ingroup API_GRAB
/// \~chinese
/// \brief ���������ѻ��������֡
/// \param [in] hCamera ����ľ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Clear all cached frames in the camera
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraClearBuffer(
	CameraHandle hCamera
);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief ִ��������
/// \param [in] hCamera ����ľ����
/// \param [in] uFlags ���ܱ�־,���@link #emCameraSoftTriggerExFlags @endlink�еĶ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \see CameraSoftTrigger
/// \~english
/// \brief Perform a soft trigger.
/// \param [in] hCamera Camera handle.
/// \param [in] uFlags function flags, as defined in @link #emCameraSoftTriggerExFlags @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSoftTrigger
MVSDK_API CameraSdkStatus __stdcall CameraSoftTriggerEx(
	CameraHandle hCamera,
	UINT uFlags
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���������HDR����Ҫ���֧�֣�����HDR���ܵ��ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] value	HDRϵ������Χ0.0��1.0
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Setting the HDR of the camera requires camera support. Models without the HDR function. This function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] value HDR coefficient, range 0.0 to 1.0
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetHDR(
	CameraHandle    hCamera,
	float           value
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ�����HDR����Ҫ���֧�֣�����HDR���ܵ��ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [out] value	HDRϵ������Χ0.0��1.0
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get camera HDR, need camera support, model without HDR function, this function returns an error code, indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [out] value HDR coefficient, range 0.0 to 1.0
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetHDR(
	CameraHandle    hCamera,
	float* value
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ��ǰ֡��ID�������֧��(����ȫϵ��֧��)���˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [out] id		   ֡ID
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief The ID of the current frame needs to be supported by the camera (supported by the full range of network ports). This function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [out] id Frame ID
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetFrameID(
	CameraHandle    hCamera,
	UINT* id
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ��ǰ֡��ʱ���(��λ΢��)
/// \param [in] hCamera ����ľ����
/// \param [out] TimeStampL   ʱ�����32λ
/// \param [out] TimeStampH   ʱ�����32λ
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the timestamp of the current frame (in microseconds)
/// \param [in] hCamera Camera handle.
/// \param [out] TimeStampL timestamp low 32 bits
/// \param [out] TimeStampH Timestamp high 32 bits
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetFrameTimeStamp(
	CameraHandle    hCamera,
	UINT* TimeStampL,
	UINT* TimeStampH
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �������������ģʽ����Ҫ���֧�֣���������ģʽ�л����ܵ��ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [in] value		   0��������    1��������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Setting the camera's gain mode requires camera support. Models without the gain mode switching function. This function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [in] value 0: low gain 1: high gain
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetHDRGainMode(
	CameraHandle    hCamera,
	int				value
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ���������ģʽ����Ҫ���֧�֣���������ģʽ�л����ܵ��ͺţ��˺������ش�����룬��ʾ��֧�֡�
/// \param [in] hCamera ����ľ����
/// \param [out] value	0��������    1��������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get camera's gain mode, need camera support, model without gain mode switching function, this function returns error code, indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [out] value 0: low gain 1: high gain
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetHDRGainMode(
	CameraHandle    hCamera,
	int* value
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ��֡���ݴ���HBITMAP
/// \param [in] hDC Handle to a device context��WIN32 API CreateDIBitmap�Ĳ���hdc��
/// \param [in] pFrameBuffer ֡����
/// \param [in] pFrameHead ֡ͷ
/// \param [out] outBitmap �´�����HBITMAP��ʹ�������Ҫ����WIN32 API DeleteObject�ͷţ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Create HBITMAP from Frame Data
/// \param [in] hDC Handle to a device context (parameter hdc of WIN32 API CreateDIBitmap)
/// \param [in] pFrameBuffer Frame data
/// \param [in] pFrameHead Frame Header
/// \param [out] outBitmap newly created HBITMAP (need to call WIN32 API DeleteObject after use)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCreateDIBitmap(
	HDC hDC,
	BYTE* pFrameBuffer,
	tSdkFrameHead* pFrameHead,
	HBITMAP* outBitmap
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ����֡��ָ������
/// \param [in] pFrameBuffer ֡����
/// \param [in] pFrameHead ֡ͷ
/// \param [in] hWnd Ŀ�Ĵ���
/// \param [in] Algorithm �����㷨  0�����ٵ������Բ�  1���ٶ�����������
/// \param [in] Mode ����ģʽ   0: �ȱ�����  1����������
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Draw frames to the specified window
/// \param [in] pFrameBuffer frame data
/// \param [in] pFrameHead Frame Header
/// \param [in] hWnd destination window
/// \param [in] Algorithm scaling algorithm 0: fast but slightly worse quality 1 slower but better quality
/// \param [in] Mode Zoom Mode 0: Scale 1: Scale Zoom
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraDrawFrameBuffer(
	BYTE* pFrameBuffer,
	tSdkFrameHead* pFrameHead,
	HWND hWnd,
	int Algorithm,
	int Mode
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ��ת֡����
/// \param [inout] pFrameBuffer ֡����
/// \param [in] pFrameHead ֡ͷ
/// \param [in] Flags 1:����   2������    3�����¡����ҽ���һ�η�ת(�൱����ת180��)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Flip frame data
/// \param [inout] pFrameBuffer frame data
/// \param [in] pFrameHead Frame Header
/// \param [in] Flags 1: Up and down 2: Around 3: Up and down, left and right are all flipped (equivalent to 180 degrees rotation)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraFlipFrameBuffer(
	BYTE* pFrameBuffer,
	tSdkFrameHead* pFrameHead,
	int Flags
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ת��֡���ݸ�ʽ
/// \param [in] hCamera ����ľ����
/// \param [in] pInFrameBuffer ����֡����
/// \param [out] pOutFrameBuffer ���֡����
/// \param [in] outWidth ������
/// \param [in] outHeight ����߶�
/// \param [in] outMediaType �����ʽ @see CameraSetIspOutFormat
/// \param [inout] pFrameHead ֡ͷ��Ϣ��ת���ɹ����������Ϣ�ᱻ�޸�Ϊ���֡����Ϣ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Conversion frame data format
/// \param [in] hCamera Camera handle.
/// \param [in] pInFrameBuffer input frame data
/// \param [out] pOutFrameBuffer output frame data
/// \param [in] outWidth output width
/// \param [in] outHeight output height
/// \param [in] outMediaType output format @see CameraSetIspOutFormat
/// \param [inout] pFrameHead frame header information (after successful conversion, the information inside will be modified to output frame information)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraConvertFrameBufferFormat(
	CameraHandle hCamera,
	BYTE* pInFrameBuffer,
	BYTE* pOutFrameBuffer,
	int outWidth,
	int outHeight,
	UINT outMediaType,
	tSdkFrameHead* pFrameHead
);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief �����������״̬�ı�Ļص�֪ͨ��������������ߡ�����ʱ��pCallBack��ָ��Ļص������ͻᱻ���á� 
/// \param [in] hCamera ����ľ����
/// \param [in] pCallBack �ص�����ָ�롣
/// \param [in] pContext  �ص������ĸ��Ӳ������ڻص�����������ʱ�ø��Ӳ����ᱻ���룬����ΪNULL��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Sets the callback notification function for camera connection state changes. When the camera is disconnected and reconnected, the callback function pointed to by pCallBack will be called.
/// \param [in] hCamera Camera handle.
/// \param [in] pCallBack callback function pointer.
/// \param [in] pContext Additional parameter of the callback function. This additional parameter will be passed in when the callback function is called. It can be NULL.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetConnectionStatusCallback(
	CameraHandle        hCamera,
	CAMERA_CONNECTION_STATUS_CALLBACK pCallBack,
	PVOID               pContext
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���ù�Դ�����������ģʽ���������ϵ������ҪӲ��֧�֣�
/// \param [in] hCamera ����ľ����
/// \param [in] index ����������
/// \param [in] mode ���ģʽ��0:��������� 1:�ֶ���
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the output mode of the light controller (Smart camera series and hardware support required)
/// \param [in] hCamera Camera handle.
/// \param [in] index controller index
/// \param [in] mode output mode (0: follow strobe 1: manual)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLightingControllerMode(
	CameraHandle        hCamera,
	int					index,
	int					mode
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ���ù�Դ�����������״̬���������ϵ������ҪӲ��֧�֣�
/// \param [in] hCamera ����ľ����
/// \param [in] index ����������
/// \param [in] state ���״̬��0:�ر�  1���򿪣�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the output status of the light controller (Smart camera series and hardware support required)
/// \param [in] hCamera Camera handle.
/// \param [in] index controller index
/// \param [in] state output state (0: off 1: on)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetLightingControllerState(
	CameraHandle        hCamera,
	int					index,
	int					state
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��������ڴ���ģʽ��������Ӳ������ʱ���������һ֡��PC�������δ�յ�PC�˵Ľ���ȷ�ϣ�������԰�֡�ط����Ρ��ñ�������������ط������������������֧�֣�
/// \param [in] hCamera ����ľ����
/// \param [in] count �ط�������<=0��ʾ�����ط����ܣ�
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief When the camera is in the trigger mode (soft trigger or hard trigger), the camera sends a frame to the PC. If the camera does not receive the reception confirmation from the PC, the camera can retransmit the frame several times. Use this function to set the number of camera resends. (only supported by Gige camera)
/// \param [in] hCamera Camera handle.
/// \param [in] count number of resends (<=0 means disable resends)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetFrameResendCount(
	CameraHandle        hCamera,
	int					count
);

/// @ingroup API_UNDISTORT
/// \~chinese
/// \brief ����У������
/// \param [in] hCamera ����ľ����
/// \param [in] width ͼƬ���
/// \param [in] height ͼƬ�߶�
/// \param [in] cameraMatrix �ڲ�(fx, fy, cx, cy)
/// \param [in] distCoeffs ����ϵ��(k1,k2,p1,p2,k3)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set undistort parameters
/// \param [in] hCamera Camera handle.
/// \param [in] width image width
/// \param [in] height image height
/// \param [in] cameraMatrix internal matrix(fx, fy, cx, cy)
/// \param [in] distCoeffs distortion coefficient (k1, k2, p1, p2, k3)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetUndistortParams(
	CameraHandle	hCamera,
	int				width,
	int				height,
	double			cameraMatrix[4],
	double			distCoeffs[5]
);

/// @ingroup API_UNDISTORT
/// \~chinese
/// \brief ��ȡУ������
/// \param [in] hCamera ����ľ����
/// \param [out] width ͼƬ���
/// \param [out] height ͼƬ�߶�
/// \param [out] cameraMatrix �ڲ�(fx, fy, cx, cy)
/// \param [out] distCoeffs ����ϵ��(k1,k2,p1,p2,k3)
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get undistort parameters
/// \param [in] hCamera Camera handle.
/// \param [out] width image width
/// \param [out] height image height
/// \param [out] cameraMatrix internal matrix(fx, fy, cx, cy)
/// \param [out] distCoeffs distortion coefficient (k1, k2, p1, p2, k3)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetUndistortParams(
	CameraHandle	hCamera,
	int* width,
	int* height,
	double			cameraMatrix[4],
	double			distCoeffs[5]
);

/// @ingroup API_UNDISTORT
/// \~chinese
/// \brief ʹ�ܾ�ͷУ��
/// \param [in] hCamera ����ľ����
/// \param [in] bEnable ʹ��У��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set undistort enable status
/// \param [in] hCamera Camera handle.
/// \param [in] bEnable enable status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetUndistortEnable(
	CameraHandle	hCamera,
	BOOL			bEnable
);

/// @ingroup API_UNDISTORT
/// \~chinese
/// \brief ��ȡ��ͷУ��ʹ��״̬
/// \param [in] hCamera ����ľ����
/// \param [out] bEnable ʹ��У��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get undistort enable status
/// \param [in] hCamera Camera handle.
/// \param [out] bEnable enable status
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetUndistortEnable(
	CameraHandle	hCamera,
	BOOL* bEnable
);

/// @ingroup API_UNDISTORT
/// \~chinese
/// \brief ��У���༭���
/// \param [in] hCamera ����ľ����
/// \param [in] hParent    ���øú����Ĵ��ڵľ��������ΪNULL��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Open the undistort editing panel
/// \param [in] hCamera Camera handle.
/// \param [in] hParent The handle of the window that called the function. Can be NULL.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraCustomizeUndistort(
	CameraHandle	hCamera,
	HWND			hParent
);

/// @ingroup API_MULTI_EYE
/// \~chinese
/// \brief ��ȡ��Ŀ�����Ŀ��
/// \param [in] hCamera ����ľ����
/// \param [out] EyeCount Ŀ��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the number of eyes in the camera
/// \param [in] hCamera Camera handle.
/// \param [out] EyeCount eye count
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetEyeCount(
	CameraHandle        hCamera,
	int* EyeCount
);

/// @ingroup API_MULTI_EYE
/// \~chinese
/// \brief �Զ�Ŀ���֡�ڵ�ĳ����Ŀͼ��ISP
/// \param [in] hCamera ����ľ����
/// \param [in] iEyeIndex ��Ŀ������
/// \param [in] pbyIn ����ͼ�����ݵĻ�������ַ������ΪNULL�� 
/// \param [in] pInFrInfo ����ͼ�����ݵ�֡ͷ������ΪNULL�� 
/// \param [out] pbyOut �����ͼ������Ļ�������ַ������ΪNULL��
/// \param [out] pOutFrInfo �����ͼ���֡ͷ��Ϣ������ΪNULL�� 
/// \param [in] uOutFormat �������ͼ��������ʽ��
/// \param [in] uReserved Ԥ����������������Ϊ0��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Do ISP for a certain monocular in the multi-camera frame.
/// \param [in] hCamera Camera handle.
/// \param [in] iEyeIndex eye index.
/// \param [in] pbyIn Input the buffer address of the image data, which cannot be NULL.
/// \param [in] pInFrInfo Input the frame header of the image data, which cannot be NULL.
/// \param [out] pbyOut The buffer address of the image output after processing, cannot be NULL.
/// \param [out] pOutFrInfo The header information of the processed image cannot be NULL.
/// \param [in] uOutFormat The output format of the image after processing.
/// \param [in] uReserved Reservation parameters must be set to 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraMultiEyeImageProcess(
	CameraHandle        hCamera,
	int					iEyeIndex,
	BYTE* pbyIn,
	tSdkFrameHead* pInFrInfo,
	BYTE* pbyOut,
	tSdkFrameHead* pOutFrInfo,
	UINT                uOutFormat,
	UINT                uReserved
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief ���������ƽ���Ҷ�ֵ
/// \param [in] pFrameBuffer ֡����
/// \param [in] pFrameHead ֡ͷ
/// \param [in] Left �����������ʼx����
/// \param [in] Top �����������ʼy����
/// \param [in] Width ��������Ŀ��
/// \param [in] Height ��������ĸ߶�
/// \param [out] AvgGray ���ؼ�����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note Width����HeightΪ0�����Left��Top��������֡��ƽ���Ҷ�ֵ
/// \~english
/// \brief Calculate the average gray value of the area
/// \param [in] pFrameBuffer frame data
/// \param [in] pFrameHead Frame Header
/// \param [in] Left The starting x coordinate of the rectangular area
/// \param [in] Top The starting y coordinate of the rectangular area
/// \param [in] Width The width of the rectangular area
/// \param [in] Height The Height of the rectangular area
/// \param [out] AvgGray returns the result of the calculation
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Width or Height is 0, then ignore Left, Top and return the average gray value of the entire frame
MVSDK_API CameraSdkStatus __stdcall CameraGetRegionAverageGray(
	BYTE* pFrameBuffer,
	tSdkFrameHead* pFrameHead,
	int Left,
	int Top,
	int Width,
	int Height,
	int* AvgGray
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ�����ʽ������֧�֡������磺H264��H265֧���������ʣ�
/// \param [in] hCamera ����ľ����
/// \param [in] iMediaType �����ʽ����
/// \param [out] uCap ����֧��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the feature support of the output format. (For example: H264, H265 support setting bit rate)
/// \param [in] hCamera Handle of the camera.
/// \param [in] iMediaType output format index
/// \param [out] uCap feature support
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetMediaCapability(
	CameraHandle    hCamera,
	int				iMediaType,
	UINT* uCap
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �������ʡ��������������ʽ֧�֣����磺H264��H265��
/// \param [in] hCamera ����ľ����
/// \param [in] iMediaType �����ʽ����
/// \param [in] uRate ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Set the bit rate. (Only some output formats are supported, such as H264, H265)
/// \param [in] hCamera Handle of the camera.
/// \param [in] iMediaType output format index
/// \param [in] uRate bit rate
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraSetMediaBitRate(
	CameraHandle    hCamera,
	int				iMediaType,
	UINT			uRate
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief ��ȡ�������á��������������ʽ֧�֣����磺H264��H265��
/// \param [in] hCamera ����ľ����
/// \param [in] iMediaType �����ʽ����
/// \param [out] uRate ����
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \~english
/// \brief Get the bit rate. (Only some output formats are supported, such as H264, H265)
/// \param [in] hCamera Handle of the camera.
/// \param [in] iMediaType output format index
/// \param [out] uRate bit rate
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus __stdcall CameraGetMediaBitRate(
	CameraHandle    hCamera,
	int				iMediaType,
	UINT* uRate
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief �������֡�¼��ص���������֡��ʼ�Լ�֡���ʱ��pCallBack��ָ��Ļص������ͻᱻ���á� 
/// \param [in] hCamera ����ľ����
/// \param [in] pCallBack �ص�����ָ�롣
/// \param [in] pContext  �ص������ĸ��Ӳ������ڻص�����������ʱ�ø��Ӳ����ᱻ���룬����ΪNULL��
/// \return �ɹ����� CAMERA_STATUS_SUCCESS(0)�����򷵻ط�0ֵ�Ĵ�����, ��ο� CameraStatus.h �д�����Ķ��塣
/// \note ����ȫ�ֿ������֡��ʼ��ʾһ֡�ع����
/// \~english
/// \brief Set the camera frame event callback function. When the frame starts and when the frame is completed, the callback function pointed to by pCallBack will be called.
/// \param [in] hCamera Camera handle.
/// \param [in] pCallBack callback function pointer.
/// \param [in] pContext Additional parameter of the callback function. This additional parameter will be passed in when the callback function is called. It can be NULL.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note For the start of the global shutter camera frame, it means the end of a frame exposure
MVSDK_API CameraSdkStatus __stdcall CameraSetFrameEventCallback(
	CameraHandle        hCamera,
	CAMERA_FRAME_EVENT_CALLBACK pCallBack,
	PVOID               pContext
);

#endif
