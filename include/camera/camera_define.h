#pragma once
#ifndef _CAMERA_DEFINE_H_
#define _CAMERA_DEFINE_H_

#include "camera_status.h"
#include <windows.h>

#define MAX_CROSS_LINE 9

/// @ingroup MV_TYPEDEF
/// \~chinese ����ľ�����Ͷ���
/// \~english Camera handle type definition
typedef int CameraHandle;



/// @ingroup MV_ENUM_TYPE
/// \~chinese ͼ����任�ķ�ʽ
/// \~english Image lookup table transformation
typedef enum
{
    LUTMODE_PARAM_GEN=0, ///< \~chinese ͨ�����ڲ�����̬����LUT��	\~english Dynamically generate LUT tables by adjusting parameters.
    LUTMODE_PRESET=1,    ///< \~chinese ʹ��Ԥ���LUT��				\~english Use a preset LUT table
    LUTMODE_USER_DEF=2	 ///< \~chinese ʹ���û��Զ����LUT��			\~english Use a user-defined LUT table
}emSdkLutMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese �������Ƶ������
/// \~english Camera video flow control
typedef enum
{
    /// \~chinese ����Ԥ��������ͼ�����ʾ�������������ڴ���ģʽ�����ȴ�����֡�ĵ�����
    /// \~english Normal preview, the captured image is displayed. (If the camera is in trigger mode, it will wait for the arrival of the trigger frame)
    RUNMODE_PLAY=0,
    RUNMODE_PAUSE=1,     ///< \~chinese ��ͣ������ͣ�����ͼ�������ͬʱҲ����ȥ����ͼ��	\~english Pause, will pause the camera's image output, and will not capture the image.
    RUNMODE_STOP=2       ///< \~chinese ֹͣ�������������ʼ��������ʹ���ֹͣģʽ		\~english Stop the camera. Camera is in stop mode after deinitialization.
}emSdkRunMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese SDK�ڲ���ʾ�ӿڵ���ʾ��ʽ
/// \~english How to display the internal display interface of the SDK
typedef enum
{
    DISPLAYMODE_SCALE=0, ///< \~chinese ������ʾģʽ�����ŵ���ʾ�ؼ��ĳߴ� \~english Zoom the display mode, zoom to the size of the display control
    DISPLAYMODE_REAL=1,  ///< \~chinese 1:1��ʾģʽ����ͼ��ߴ������ʾ�ؼ��ĳߴ�ʱ��ֻ��ʾ�ֲ�  \~english 1:1 display mode, when the image size is larger than the size of the display control, only the local display
    DISPLAYMODE_2X=2,	 ///< \~chinese �Ŵ�2X	\~english Zoom in 2X
    DISPLAYMODE_4X=3,	 ///< \~chinese �Ŵ�4X	\~english Zoom in 4X
    DISPLAYMODE_8X=4,    ///< \~chinese �Ŵ�8X	\~english Zoom in 8X
    DISPLAYMODE_16X=5,	 ///< \~chinese �Ŵ�16X	\~english Zoom in 16X
    DISPLAYMODE_SCALE_FIT=6	///< \~chinese �������ţ�������ʾ����	\~english Stretch zoom to fill the display area
}emSdkDisplayMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ¼��״̬
/// \~english Recording status
typedef enum
{
    RECORD_STOP=0,	///< \~chinese ֹͣ		\~english Stop
    RECORD_START=1,   ///< \~chinese ¼����	\~english Start
    RECORD_PAUSE=2    ///< \~chinese ��ͣ		\~english Pause
}emSdkRecordMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ͼ��ľ������
/// \~english Image mirroring operation
typedef enum
{
    MIRROR_DIRECTION_HORIZONTAL=0,		///< \~chinese ˮƽ����	\~english Horizontal mirroring
    MIRROR_DIRECTION_VERTICAL=1			///< \~chinese ��ֱ����	\~english Vertical mirroring
}emSdkMirrorDirection;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ͼ�����ת����
/// \~english Rotation of the image
typedef enum
{
    ROTATE_DIRECTION_0=0,		///< \~chinese ����ת		\~english Do not rotate
    ROTATE_DIRECTION_90=1,		///< \~chinese ��ʱ��90��	\~english Counterclockwise 90 degrees
    ROTATE_DIRECTION_180=2,		///< \~chinese ��ʱ��180��	\~english Counterclockwise 180 degrees
    ROTATE_DIRECTION_270=3,		///< \~chinese ��ʱ��270��	\~english Counterclockwise 270 degrees
}emSdkRotateDirection;

/// @ingroup MV_ENUM_TYPE
/// \~chinese �����Ƶ��֡��
/// \~english Camera video frame rate
typedef enum
{
    FRAME_SPEED_LOW=0,		///< \~chinese ����ģʽ	\~english Low Speed
    FRAME_SPEED_NORMAL=1,   ///< \~chinese ��ͨģʽ	\~english Normal Speed
    FRAME_SPEED_HIGH=2,     ///< \~chinese ����ģʽ(��Ҫ�ϸߵĴ������,���豸���������ʱ���֡�ʵ��ȶ�����Ӱ��)	\~english High Speed
    FRAME_SPEED_SUPER=3     ///< \~chinese ������ģʽ(��Ҫ�ϸߵĴ������,���豸���������ʱ���֡�ʵ��ȶ�����Ӱ��)	\~english Super Speed
}emSdkFrameSpeed;

/// @ingroup MV_ENUM_TYPE
/// \~chinese �����ļ��ĸ�ʽ����
/// \~english Save file format type
typedef enum
{
    FILE_JPG = 1,	///< \~chinese JPG				\~english JPG
    FILE_BMP = 2,	///< \~chinese BMP 24bit		\~english BMP 24bit
    FILE_RAW = 4,	///< \~chinese RAW				\~english RAW
    FILE_PNG = 8,	///< \~chinese PNG 24bit		\~english PNG 24bit
    FILE_BMP_8BIT = 16,	///< \~chinese BMP 8bit		\~english BMP 8bit
    FILE_PNG_8BIT = 32, ///< \~chinese PNG 8bit		\~english PNG 8bit
    FILE_RAW_16BIT = 64	///< \~chinese RAW 16bit	\~english RAW 16bit
}emSdkFileType;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ����е�ͼ�񴫸����Ĺ���ģʽ
/// \~english Image Sensor Operation Mode in Camera
typedef enum
{
    /// \~chinese �����ɼ�ģʽ
    /// \~english Continuous acquisition mode
    CONTINUATION=0,

    /// \~chinese �������ģʽ�����������ָ��󣬴�������ʼ�ɼ�ָ��֡����ͼ�񣬲ɼ���ɺ�ֹͣ���
    /// \~english Software trigger mode. After the software sends the instruction, the sensor starts to capture the image of the specified frame number. After the acquisition is completed, the output is stopped.
    SOFT_TRIGGER=1,

    /// \~chinese Ӳ������ģʽ�������յ��ⲿ�źţ���������ʼ�ɼ�ָ��֡����ͼ�񣬲ɼ���ɺ�ֹͣ���
    /// \~english In the hardware trigger mode, when receiving an external signal, the sensor starts to capture the image of the specified frame number. After the acquisition is completed, the output is stopped.
    EXTERNAL_TRIGGER=2,

    /// \~chinese ����������ģʽ�����������������
    /// \~english Encoder trigger mode (only for line scan cameras)
    ROTARYENC_TRIGGER=3,

    /// \~chinese ��������������ģʽ�����������������
    /// \~english Encoder condition trigger mode (only for line scan cameras)
    ROTARYENC_COND_TRIGGER=4,
    } emSdkSnapMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese �Զ��ع�ʱ��Ƶ����Ƶ��
/// \~english Anti-strobe frequency at auto exposure
typedef enum
{
    /// \~chinese 50HZ,һ��ĵƹⶼ��50HZ
    /// \~english 50HZ, the general lighting is 50HZ
    LIGHT_FREQUENCY_50HZ=0,

    /// \~chinese 60HZ,��Ҫ��ָ��ʾ����
    /// \~english 60HZ, mainly refers to the monitor
    LIGHT_FREQUENCY_60HZ=1
}emSdkLightFrequency;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ��������ò�������ΪA,B,C,D 4����б��档
/// \~english The camera configuration parameters are divided into groups A, B, C, and D for preservation.
typedef enum
{
    PARAMETER_TEAM_DEFAULT = 0xff,	///< \~chinese Ĭ�ϲ��� \~english Default parameters
    PARAMETER_TEAM_A = 0,	///< \~chinese ����A \~english parametersA
    PARAMETER_TEAM_B = 1,	///< \~chinese ����B \~english parametersB
    PARAMETER_TEAM_C = 2,	///< \~chinese ����C \~english parametersC
    PARAMETER_TEAM_D = 3	///< \~chinese ����D \~english parametersD
}emSdkParameterTeam;


/// @ingroup MV_ENUM_TYPE
/// \~chinese
/// \brief �����������ģʽ
/// \note �����Ը����Լ���ʹ�û��������ʹ�����ϼ��ַ�ʽ���ز��������磬��
/// \note MV-U300Ϊ������ϣ����̨���ͺŵ���������� �����϶�����4���������ô��
/// \note ʹ��PARAM_MODE_BY_MODEL��ʽ;�����ϣ������ĳһ̨����ĳ��̨MV-U300��
/// \note ʹ���Լ������ļ��������MV-U300��Ҫʹ����ͬ�Ĳ����ļ�����ôʹ��
/// \note PARAM_MODE_BY_NAME��ʽ;�����ϣ��ÿ̨MV-U300��ʹ�ò�ͬ�Ĳ����ļ�����ô
/// \note ʹ��PARAM_MODE_BY_SN��ʽ��
/// \note �����ļ����ڰ�װĿ¼�� \\Camera\\Configs Ŀ¼�£���configΪ��׺�����ļ���
/// \~english
/// \brief Camera parameter loading mode
/// \note You can use the above several ways to load parameters according to your own environment. For example, with
/// \note MV-U300 as an example, you want multiple cameras of this model to share 4 sets of parameters on your computer.
/// \note Use the PARAM_MODE_BY_MODEL method; if you want one or more of the MV-U300s
/// \note Use your own parameter file and the rest of the MV-U300 use the same parameter file again, then use
/// \note PARAM_MODE_BY_NAME way; if you want each MV-U300 to use a different parameter file, then
/// \note Use the PARAM_MODE_BY_SN method.
/// \note The parameter file exists in the \\Camera\\Configs directory of the installation directory, with a config extension file.
typedef enum
{
    /// \~chinese ��������ͺ������ļ��м��ز���������MV-U300
    /// \note ����ͬ�ͺŵ��������ABCD��������ļ����޸�һ̨����Ĳ����ļ�����Ӱ�쵽����ͬ�ͺŵ�����������ء�
    /// \~english Load parameters from a file based on the camera model name, such as the MV-U300
    /// \note All ABCD four-group parameter files are shared by all cameras of the same model. Modifying a camera's parameter file will affect the entire camera model parameter loading.
    PARAM_MODE_BY_MODEL=0,

    /// \~chinese �����豸�ǳ�(tSdkCameraDevInfo.acFriendlyName)���ļ��м��ز���������MV-U300,���ǳƿ��Զ���
    /// \note �����豸����ͬ�����������ABCD��������ļ���
    /// \note Ĭ������£���������ֻ����ĳ�ͺ�һ̨���ʱ��
    /// \note �豸������һ���ģ�����ϣ��ĳһ̨����ܹ�����
    /// \note ��ͬ�Ĳ����ļ��������ͨ���޸����豸���ķ�ʽ
    /// \note ���������ָ���Ĳ����ļ���
    /// \~english Load parameters from a file based on device nickname (tSdkCameraDevInfo.acFriendlyName), such as MV-U300, which can be customized
    /// \note All cameras with the same device name share the four ABCD parameter files.
    /// \note By default, when only one model of a camera is connected to the computer,
    /// \note The device name is the same, and you want a camera to load
    /// \note different parameter files, you can modify the device name
    /// \note to have it load the specified parameter file.
    PARAM_MODE_BY_NAME=1,

    /// \~chinese �����豸��Ψһ���кŴ��ļ��м��ز��������к��ڳ���ʱ�Ѿ�д���豸��ÿ̨���ӵ�в�ͬ�����кš�
    /// \note ��������Լ���Ψһ���к�������ABCD��������ļ���
    /// \note ���к��ڳ���ʱ�Ѿ��̻�������ڣ�ÿ̨��������к�
    /// \note ������ͬ��ͨ�����ַ�ʽ��ÿ̨����Ĳ����ļ����Ƕ����ġ�
    /// \~english The parameters are loaded from the file according to the unique serial number of the device. The serial number is already written to the device at the factory and each camera has a different serial number.
    /// \note The camera loads ABCD four sets of parameter files according to their unique serial number.
    /// \note The serial number has been fixed in the camera at the factory, the serial number of each camera
    /// \note is not the same. In this way, the parameter files for each camera are independent.
    PARAM_MODE_BY_SN=2,

    /// \~chinese ���豸�Ĺ�̬�洢���м��ز������������е��ͺŶ�֧�ִ�����ж�д�����飬��tSdkCameraCapbility.bParamInDevice����
    /// \~english Load parameters from the device's solid-state memory. Not all models support reading and writing parameters from the camera, as determined by tSdkCameraCapbility.bParamInDevice
    PARAM_MODE_IN_DEVICE=3
}emSdkParameterMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese SDK���ɵ��������ҳ������ֵ
/// \~english SDK generated camera configuration page mask values
typedef enum
{
    PROP_SHEET_INDEX_EXPOSURE=0,			///< \~chinese �ع����� \~english Exposure Settings
    PROP_SHEET_INDEX_ISP_COLOR=1,			///< \~chinese ��ɫ�������� \~english Color Matrix Settings
    PROP_SHEET_INDEX_ISP_LUT=2,			///< \~chinese LUT���� \~english LUT setting
    PROP_SHEET_INDEX_ISP_SHAPE=3,			///< \~chinese �任���� \~english transform settings
    PROP_SHEET_INDEX_VIDEO_FORMAT=4,		///< \~chinese ��ʽ���� \~english Formatting
    PROP_SHEET_INDEX_RESOLUTION=5,		///< \~chinese �ֱ������� \~english resolution setting
    PROP_SHEET_INDEX_IO_CTRL=6,			///< \~chinese IO���� \~english IO control
    PROP_SHEET_INDEX_TRIGGER_SET=7,		///< \~chinese ����ģʽ \~english trigger setting
    PROP_SHEET_INDEX_OVERLAY=8,			///< \~chinese ʮ���� \~english Crosshair
    PROP_SHEET_INDEX_DEVICE_INFO=9,		///< \~chinese �豸��Ϣ \~english Device Information
    PROP_SHEET_INDEX_WDR=10,				///< \~chinese ��̬ \~english Wide Dynamic
    PROP_SHEET_INDEX_MULTI_EXPOSURE=11,	///< \~chinese �����ع� \~english Multi exposure
    PROP_SHEET_INDEX_SPECIAL=12,			///< \~chinese �������� \~english Special settings
}emSdkPropSheetMask;

/// @ingroup MV_ENUM_TYPE
/// \~chinese SDK���ɵ��������ҳ��Ļص���Ϣ����
/// \~english SDK callback camera configuration page callback message type
typedef enum
{
    SHEET_MSG_LOAD_PARAM_DEFAULT=0,	///< \~chinese �������ָ���Ĭ�Ϻ󣬴�������Ϣ \~english After the parameter is restored to the default, the message is triggered
    SHEET_MSG_LOAD_PARAM_GROUP=1,     ///< \~chinese ����ָ�������飬��������Ϣ \~english Load the specified parameter group and trigger the message
    SHEET_MSG_LOAD_PARAM_FROMFILE=2,  ///< \~chinese ��ָ���ļ����ز����󣬴�������Ϣ \~english Fires the message after loading parameters from the specified file
    SHEET_MSG_SAVE_PARAM_GROUP=3      ///< \~chinese ��ǰ�����鱻����ʱ����������Ϣ \~english Trigger this message when the current parameter group is saved
}emSdkPropSheetMsg;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ���ӻ�ѡ��ο����ڵ�����
/// \~english Visualize the type of reference window
typedef enum
{
    REF_WIN_AUTO_EXPOSURE=0,	///< \~chinese �Զ��عⴰ�� \~english Automatic exposure window
    REF_WIN_WHITE_BALANCE=1,	///< \~chinese ��ƽ�ⴰ�� \~english White balance window
}emSdkRefWinType;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ���ӻ�ѡ��ο����ڵ�����
/// \~english Visualize the type of reference window
typedef enum
{
    RES_MODE_PREVIEW=0,	///< \~chinese Ԥ�� \~english Preview
    RES_MODE_SNAPSHOT=1,	///< \~chinese ץ�� \~english Snapshot
}emSdkResolutionMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ��ƽ��ʱɫ��ģʽ
/// \~english White balance color temperature mode
typedef enum
{
    CT_MODE_AUTO=0,		///< \~chinese �Զ�ʶ��ɫ�� \~english Automatically recognize color temperature
    CT_MODE_PRESET=1,		///< \~chinese ʹ��ָ����Ԥ��ɫ�� \~english Use the specified preset color temperature
    CT_MODE_USER_DEF=2	///< \~chinese �Զ���ɫ��(����;���) \~english Custom color temperature (gain and matrix)
}emSdkClrTmpMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese LUT����ɫͨ��
/// \~english LUT color channel
typedef enum
{
    LUT_CHANNEL_ALL=0,	///< \~chinese R,B,G��ͨ��ͬʱ���� \~english R, B, G simultaneous adjustment of three channels
    LUT_CHANNEL_RED=1,	///< \~chinese ��ɫͨ�� \~english Red channel
    LUT_CHANNEL_GREEN=2,	///< \~chinese ��ɫͨ�� \~english Green channel
    LUT_CHANNEL_BLUE=3,	///< \~chinese ��ɫͨ�� \~english Blue channel
}emSdkLutChannel;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ISP����Ԫ
/// \~english ISP processing unit
typedef enum
{
    ISP_PROCESSSOR_PC=0,		///< \~chinese ʹ��PC�����ISPģ�� \~english Use software ISP module of PC
    ISP_PROCESSSOR_DEVICE=1	///< \~chinese ʹ������Դ���Ӳ��ISPģ�� \~english Use the camera's own hardware ISP module
}emSdkIspProcessor;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ������źſ��Ʒ�ʽ
/// \~english Strobe signal control method
typedef enum
{
    STROBE_SYNC_WITH_TRIG_AUTO=0,		///< \~chinese �ʹ����ź�ͬ������������������ع�ʱ���Զ�����STROBE�źš���ʱ����Ч���Կ�����(@link #CameraSetStrobePolarity @endlink)�� \~english Synchronized with the trigger signal, the STROBE signal is automatically generated when the camera performs exposure. At this point, the effective polarity can be set (@link #CameraSetStrobePolarity @endlink).
    STROBE_SYNC_WITH_TRIG_MANUAL=1,   ///< \~chinese �ʹ����ź�ͬ����������STROBE��ʱָ����ʱ���(@link #CameraSetStrobeDelayTime @endlink)���ٳ���ָ��ʱ�������(@link #CameraSetStrobePulseWidth @endlink)����Ч���Կ�����(@link #CameraSetStrobePolarity @endlink)�� \~english Synchronized with the trigger signal. After the trigger, STROBE is delayed by the specified time (@link #CameraSetStrobeDelayTime @endlink) and continues for the specified time (@link #CameraSetStrobePulseWidth @endlink). The effective polarity can be set (@link #CameraSetStrobePolarity @endlink).
    STROBE_ALWAYS_HIGH=2,             ///< \~chinese ʼ��Ϊ�ߣ�����STROBE�źŵ��������� \~english Always high, ignoring other settings of the STROBE signal
    STROBE_ALWAYS_LOW=3               ///< \~chinese ʼ��Ϊ�ͣ�����STROBE�źŵ��������� \~english Always low, ignoring other settings of the STROBE signal
}emStrobeControl;

/// @ingroup MV_ENUM_TYPE
/// \~chinese Ӳ���ⴥ�����ź�����
/// \~english Signal types triggered by hardware
typedef enum
{
    EXT_TRIG_LEADING_EDGE=0,		///< \~chinese �����ش�����Ĭ��Ϊ�÷�ʽ \~english	Rising edge trigger, default is this method
    EXT_TRIG_TRAILING_EDGE=1,		///< \~chinese �½��ش��� \~english	Falling edge trigger
    EXT_TRIG_HIGH_LEVEL=2,			///< \~chinese �ߵ�ƽ����,��ƽ��Ⱦ����ع�ʱ�䣬�������ͺŵ����֧�ֵ�ƽ������ʽ�� \~english The high level triggers, the level width determines the exposure time, only some models of cameras support level triggering.
    EXT_TRIG_LOW_LEVEL=3,			///< \~chinese �͵�ƽ���� \~english	Low level trigger
    EXT_TRIG_DOUBLE_EDGE=4,			///< \~chinese ˫���ش��� \~english	Bilateral trigger
}emExtTrigSignal;

/// @ingroup MV_ENUM_TYPE
/// \~chinese Ӳ���ⴥ��ʱ�Ŀ��ŷ�ʽ
/// \~english Shutter mode when triggered by hardware
typedef enum
{
    EXT_TRIG_EXP_STANDARD=0,			///< \~chinese ��׼��ʽ��Ĭ��Ϊ�÷�ʽ�� \~english	Standard mode, the default is this mode.
    EXT_TRIG_EXP_GRR=1,				///< \~chinese ȫ�ָ�λ��ʽ�����ֹ������ŵ�CMOS�ͺŵ����֧�ָ÷�ʽ������ⲿ��е���ţ����Դﵽȫ�ֿ��ŵ�Ч�����ʺ��ĸ����˶������� \~english Global reset mode, part of the rolling shutter CMOS model camera supports this method, with the external mechanical shutter, you can achieve the effect of a global shutter, suitable for shooting high-speed objects
}emExtTrigShutterMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese �����������㷨
/// \~english Sharpness assessment algorithm
typedef enum
{
    EVALUATE_DEFINITION_DEVIATION=0,	///< \~chinese ��� \~english	Variance method
    EVALUATE_DEFINITION_SMD=1,			///< \~chinese �������ػҶȷ�� \~english Adjacent Pixel Gray Difference Method
    EVALUATE_DEFINITION_GRADIENT=2,		///< \~chinese �ݶ�ͳ�� \~english Gradient statistics
    EVALUATE_DEFINITION_SOBEL=3,		///< \~chinese Sobel \~english Sobel
    EVALUATE_DEFINITION_ROBERT=4,		///< \~chinese Robert \~english Robert
    EVALUATE_DEFINITION_LAPLACE=5,		///< \~chinese Laplace \~english Laplace

    EVALUATE_DEFINITION_ALG_MAX=6,		///< \~chinese �㷨���� \~english The number of algorithms
}emEvaluateDefinitionAlgorith;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ���������־
/// \~english Text output flag
typedef enum
{
    CAMERA_DT_VCENTER		= 0x1,		///< \~chinese ��ֱ���� \~english Vertically centered
    CAMERA_DT_BOTTOM		= 0x2,		///< \~chinese �ײ����� \~english Bottom alignment
    CAMERA_DT_HCENTER		= 0x4,		///< \~chinese ˮƽ���� \~english Horizontally centered
    CAMERA_DT_RIGHT			= 0x8,		///< \~chinese �Ҷ��� \~english	Right alignment
    CAMERA_DT_SINGLELINE	= 0x10,		///< \~chinese ������ʾ \~english Single-line display
    CAMERA_DT_ALPHA_BLEND	= 0x20,		///< \~chinese Alpha��� \~english Alpha blend
    CAMERA_DT_ANTI_ALIASING = 0x40,		///< \~chinese ����� \~english	Anti-aliasing
}emCameraDrawTextFlags;

/// @ingroup MV_ENUM_TYPE
/// \~chinese GPIOģʽ
/// \~english GPIO Mode
typedef enum
{
    IOMODE_TRIG_INPUT=0,		///< \~chinese �������� \~english Trigger input
    IOMODE_STROBE_OUTPUT=1,		///< \~chinese �������� \~english Strobe output
    IOMODE_GP_INPUT=2,			///< \~chinese ͨ�������� \~english Universal input
    IOMODE_GP_OUTPUT=3,			///< \~chinese ͨ������� \~english Universal output
    IOMODE_PWM_OUTPUT=4,		///< \~chinese PWM����� \~english PWM output
    IOMODE_ROTARYENC_INPUT=5,   ///< \~chinese ���������� \~english rotary input
}emCameraGPIOMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese GPIO ��ʽ
/// \~english GPIO Format
typedef enum
{
    IOFORMAT_SINGLE=0,			///< \~chinese ����  \~english single ended
    IOFORMAT_RS422=1,			///< \~chinese ���RS422 \~english Differential RS422
    IOFORMAT_RS422_TERM=2,		///< \~chinese ���RS422���ն˵��� \~english Differential RS422 and Termination Enable
}emCameraGPIOFormat;

/// @ingroup MV_ENUM_TYPE
/// \~chinese ȡͼ���ȼ�
/// \~english Get Image priority
typedef enum
{
    CAMERA_GET_IMAGE_PRIORITY_OLDEST=0,		///< \~chinese ��ȡ��������ɵ�һ֡ \~english	Get the oldest frame in the cache
    CAMERA_GET_IMAGE_PRIORITY_NEWEST=1,		///< \~chinese ��ȡ���������µ�һ֡���ȴ�֡�ɵĽ�ȫ�������� \~english Get the latest frame in the cache (older than this frame will be discarded)

    /// \~chinese ���������е�����֡����������˿���������ع���佫�ᱻ������ϣ��ȴ�������һ֡
    /// \note ĳЩ�ͺŵ������֧�ִ˹��ܣ����ڲ�֧�ִ˹��ܵ���������־�൱��@link #CAMERA_GET_IMAGE_PRIORITY_OLDEST @endlink
    /// \~english All frames in the cache are discarded, and if the camera is now being exposed or transmitted it will be immediately interrupted, waiting to receive the next frame
    /// \note Some models do not support this feature. For cameras that do not support this feature this flag is equivalent to @link #CAMERA_GET_IMAGE_PRIORITY_OLDEST @endlink
    CAMERA_GET_IMAGE_PRIORITY_NEXT=2,
    }emCameraGetImagePriority;

/// @ingroup MV_ENUM_TYPE
/// \~chinese �������ܱ�־
/// \~english Soft trigger function flag
typedef enum
{
    CAMERA_ST_CLEAR_BUFFER_BEFORE	= 0x1,	///< \~chinese ������֮ǰ���������ѻ����֡ \~english Empty camera-cached frames before soft triggering
}emCameraSoftTriggerExFlags;

/// \~chinese ������豸��Ϣ
/// \~english Camera device information
typedef struct
{
    char acProductSeries[32];	///< \~chinese ��Ʒϵ�� \~english Product Series
    char acProductName[32];		///< \~chinese ��Ʒ���� \~english product name

    /// \~chinese ��Ʒ�ǳƣ��û����Զ�����ǳƣ�����������ڣ��������ֶ�����ͬʱʹ��,������@link #CameraSetFriendlyName @endlink�ӿڸı���ǳƣ��豸��������Ч��
    /// \~english Product nicknames, users can customize the nickname, saved in the camera, used to distinguish between multiple cameras at the same time, you can use the @link #CameraSetFriendlyName @endlink interface to change the nickname, the device takes effect after restart.
    char acFriendlyName[32];
    char acLinkName[32];		///< \~chinese �ں˷������������ڲ�ʹ�� \~english	Kernel symbolic connection name, internal use
    char acDriverVersion[32];	///< \~chinese �����汾 \~english Driver version
    char acSensorType[32];		///< \~chinese sensor���� \~english	Sensor type
    char acPortType[32];		///< \~chinese �ӿ����� \~english Interface Type
    char acSn[32];				///< \~chinese ��ƷΨһ���к� \~english Product unique serial number
    UINT uInstance;				///< \~chinese ���ͺ�����ڸõ����ϵ�ʵ�������ţ���������ͬ�ͺŶ���� \~english The example index number of this model camera on this computer, used to distinguish the same model multiple cameras
} tSdkCameraDevInfo;

/// @ingroup MV_MACRO_TYPE
/// @{
#define EXT_TRIG_MASK_GRR_SHUTTER  1	///< \~chinese ����֧��GRRģʽ \~english Shutter supports GRR mode
#define EXT_TRIG_MASK_LEVEL_MODE   2	///< \~chinese ֧�ֵ�ƽ���� \~english Support level trigger
#define EXT_TRIG_MASK_DOUBLE_EDGE  4	///< \~chinese ֧��˫���ش��� \~english Supports bilateral triggering
#define EXT_TRIG_MASK_BUFFERED_DELAY 8	///< \~chinese ֧���źź��� \~english Supports signal delayed activation

//tSdkResolutionRange�ṹ����SKIP�� BIN��RESAMPLEģʽ������ֵ
#define MASK_2X2_HD     (1<<0)    //Ӳ��SKIP��BIN���ز��� 2X2
#define MASK_3X3_HD     (1<<1)
#define MASK_4X4_HD     (1<<2)
#define MASK_5X5_HD     (1<<3)
#define MASK_6X6_HD     (1<<4)
#define MASK_7X7_HD     (1<<5)
#define MASK_8X8_HD     (1<<6)
#define MASK_9X9_HD     (1<<7)
#define MASK_10X10_HD   (1<<8)
#define MASK_11X11_HD   (1<<9)
#define MASK_12X12_HD   (1<<10)
#define MASK_13X13_HD   (1<<11)
#define MASK_14X14_HD   (1<<12)
#define MASK_15X15_HD   (1<<13)
#define MASK_16X16_HD   (1<<14)
#define MASK_17X17_HD   (1<<15)
#define MASK_2X2_SW     (1<<16)   //���SKIP��BIN���ز��� 2X2
#define MASK_3X3_SW     (1<<17)
#define MASK_4X4_SW     (1<<18)
#define MASK_5X5_SW     (1<<19)
#define MASK_6X6_SW     (1<<20)
#define MASK_7X7_SW     (1<<21)
#define MASK_8X8_SW     (1<<22)
#define MASK_9X9_SW     (1<<23)
#define MASK_10X10_SW   (1<<24)
#define MASK_11X11_SW   (1<<25)
#define MASK_12X12_SW   (1<<26)
#define MASK_13X13_SW   (1<<27)
#define MASK_14X14_SW   (1<<28)
#define MASK_15X15_SW   (1<<29)
#define MASK_16X16_SW   (1<<30)
#define MASK_17X17_SW   (1<<31)
/// @}

/// \~chinese ����ķֱ����趨��Χ�������ڹ���UI
/// \~english Camera resolution setting range, can be used for component UI
typedef struct
{
    INT iHeightMax;			///< \~chinese ͼ�����߶� \~english Maximum image height
    INT iHeightMin;           ///< \~chinese ͼ����С�߶� \~english Image minimum height
    INT iWidthMax;            ///< \~chinese ͼ������� \~english The maximum width of the image
    INT iWidthMin;            ///< \~chinese ͼ����С��� \~english The minimum width of the image
    UINT uSkipModeMask;       ///< \~chinese SKIPģʽ���룬Ϊ0����ʾ��֧��SKIP ��bit0Ϊ1,��ʾ֧��SKIP 2x2 ;bit1Ϊ1����ʾ֧��SKIP 3x3.... \~english The SKIP mode mask, which is 0, indicates that SKIP is not supported. Bit0 is 1 to indicate that SKIP 2x2 is supported; bit1 is 1 to indicate that SKIP 3x3 is supported....
    UINT uBinSumModeMask;     ///< \~chinese BIN(���)ģʽ���룬Ϊ0����ʾ��֧��BIN ��bit0Ϊ1,��ʾ֧��BIN 2x2 ;bit1Ϊ1����ʾ֧��BIN 3x3.... \~english The BIN (sum) pattern mask, which is 0, indicates that BIN is not supported. Bit0 is 1, indicating that BIN 2x2 is supported; bit1 is 1, indicating that BIN 3x3 is supported....
    UINT uBinAverageModeMask; ///< \~chinese BIN(���ֵ)ģʽ���룬Ϊ0����ʾ��֧��BIN ��bit0Ϊ1,��ʾ֧��BIN 2x2 ;bit1Ϊ1����ʾ֧��BIN 3x3.... \~english The BIN (mean value) mode mask, which is 0, indicates that BIN is not supported. Bit0 is 1, indicating that BIN 2x2 is supported; bit1 is 1, indicating that BIN 3x3 is supported....
    UINT uResampleMask;       ///< \~chinese Ӳ���ز��������� \~english Hardware resampling mask
} tSdkResolutionRange;

/// \~chinese ����ķֱ�������
/// \~english Camera resolution description
typedef struct
{
    INT     iIndex;				///< \~chinese �����ţ�[0,N]��ʾԤ��ķֱ���(N ΪԤ��ֱ��ʵ���������һ�㲻����20),OXFF ��ʾ�Զ���ֱ���(ROI) \~english Index number, [0, N] indicates the preset resolution (N is the maximum number of preset resolutions, generally no more than 20), OXFF indicates custom resolution (ROI)
    char    acDescription[32];	///< \~chinese �÷ֱ��ʵ�������Ϣ����Ԥ��ֱ���ʱ����Ϣ��Ч���Զ���ֱ��ʿɺ��Ը���Ϣ \~english The description of the resolution. This information is valid only when the resolution is preset. Custom resolution ignores this information
    UINT    uBinSumMode;			///< \~chinese BIN(���)��ģʽ,��Χ���ܳ���tSdkResolutionRange.uBinSumModeMask \~english The BIN (sum) mode cannot exceed the tSdkResolutionRange.uBinSumModeMask
    UINT    uBinAverageMode;		///< \~chinese BIN(���ֵ)��ģʽ,��Χ���ܳ���tSdkResolutionRange.uBinAverageModeMask \~english BIN (average) mode, the range cannot exceed tSdkResolutionRange.uBinAverageModeMask
    UINT    uSkipMode;			///< \~chinese �Ƿ�SKIP�ĳߴ磬Ϊ0��ʾ��ֹSKIPģʽ����Χ���ܳ���tSdkResolutionRange.uSkipModeMask \~english Whether the SKIP size is 0 indicates that the SKIP mode is disabled and the range cannot exceed the tSdkResolutionRange.uSkipModeMask
    UINT    uResampleMask;		///< \~chinese Ӳ���ز��������� \~english Hardware resampling mask
    INT     iHOffsetFOV;			///< \~chinese �ɼ��ӳ������Sensor����ӳ����Ͻǵ�ˮƽƫ�� \~english The horizontal offset of the acquisition field of view relative to the top left corner of the Sensor's largest field of view
    INT     iVOffsetFOV;			///< \~chinese �ɼ��ӳ������Sensor����ӳ����ϽǵĴ�ֱƫ�� \~english The vertical offset of the acquisition field of view relative to the upper left corner of the Sensor's largest field of view
    INT     iWidthFOV;			///< \~chinese �ɼ��ӳ��Ŀ�� \~english The width of the field of view
    INT     iHeightFOV;			///< \~chinese �ɼ��ӳ��ĸ߶� \~english The height of the field of view
    INT     iWidth;				///< \~chinese ������������ͼ��Ŀ�� \~english The width of the final output image of the camera
    INT     iHeight;				///< \~chinese ������������ͼ��ĸ߶� \~english The height of the final output image of the camera
    INT     iWidthZoomHd;			///< \~chinese Ӳ�����ŵĿ��,����Ҫ���д˲����ķֱ��ʣ��˱�������Ϊ0. \~english The width of the hardware zoom, resolution that does not require this operation, this variable is set to 0.
    INT     iHeightZoomHd;		///< \~chinese Ӳ�����ŵĸ߶�,����Ҫ���д˲����ķֱ��ʣ��˱�������Ϊ0. \~english The height of the hardware zoom, resolution that does not require this operation, this variable is set to 0.
    INT     iWidthZoomSw;			///< \~chinese ������ŵĿ��,����Ҫ���д˲����ķֱ��ʣ��˱�������Ϊ0. \~english The width of the software's zoom, which does not require the resolution of this operation, this variable is set to 0.
    INT     iHeightZoomSw;		///< \~chinese ������ŵĸ߶�,����Ҫ���д˲����ķֱ��ʣ��˱�������Ϊ0. \~english The height of the software's zoom, which does not require the resolution of this operation, this variable is set to 0.
} tSdkImageResolution;

/// \~chinese �����ƽ��ɫ��ģʽ������Ϣ
/// \~english Camera white balance color temperature mode description information
typedef struct
{
    INT  iIndex;			///< \~chinese ģʽ������ \~english	Mode index number
    char acDescription[32];	///< \~chinese ������Ϣ \~english Description
} tSdkColorTemperatureDes;

/// \~chinese ���֡��������Ϣ
/// \~english Camera frame rate description information
typedef struct
{
    INT  iIndex;             ///< \~chinese ֡�������ţ�һ��0��Ӧ�ڵ���ģʽ��1��Ӧ����ͨģʽ��2��Ӧ�ڸ���ģʽ \~english Frame rate index number, generally 0 corresponds to low speed mode, 1 corresponds to normal mode, 2 corresponds to high speed mode
    char acDescription[32];  ///< \~chinese ������Ϣ \~english Description
} tSdkFrameSpeed;

/// \~chinese ����ع⹦�ܷ�Χ����
/// \see tSdkCameraCapbility.sExposeDesc
/// \~english Camera exposure function scope definition
/// \see tSdkCameraCapbility.sExposeDesc
typedef struct
{
    UINT  uiTargetMin;		///< \~chinese �Զ��ع�����Ŀ����Сֵ	\~english Auto exposure brightness target minimum
    UINT  uiTargetMax;		///< \~chinese �Զ��ع�����Ŀ�����ֵ	\~english Auto exposure brightness target maximum
    UINT  uiAnalogGainMin;	///< \~chinese ģ���������Сֵ����λΪfAnalogGainStep�ж��� \~english	The minimum value of the analog gain in fAnalog defined in GainStep
    UINT  uiAnalogGainMax;	///< \~chinese ģ����������ֵ����λΪfAnalogGainStep�ж��� \~english The maximum value of the analog gain in fAnalog defined in GainStep
    float fAnalogGainStep;	///< \~chinese ģ������ÿ����1����Ӧ�����ӵķŴ��������磬uiAnalogGainMinһ��Ϊ16��fAnalogGainStepһ��Ϊ0.125����ô��С�Ŵ�������16*0.125 = 2�� \~english Each increase in analog gain corresponds to an increased amplification factor. For example, uiAnalogGainMin is generally 16 and fAnalogGainStep is generally 0.125, so the minimum magnification is 16*0.125 = 2 times
    UINT  uiExposeTimeMin;	///< \~chinese �ֶ�ģʽ�£��ع�ʱ�����Сֵ����λ:�С�����CameraGetExposureLineTime���Ի��һ�ж�Ӧ��ʱ��(΢��),�Ӷ��õ���֡���ع�ʱ�� \~english The minimum exposure time in manual mode, unit: line. According to CameraGetExposureLineTime can get a row of corresponding time (microseconds) to get the entire frame exposure time
    UINT  uiExposeTimeMax;	///< \~chinese �ֶ�ģʽ�£��ع�ʱ������ֵ����λ:�� \~english Maximum exposure time in manual mode, unit: line
} tSdkExpose;

/// \~chinese ����ģʽ����
/// \~english Trigger mode description
typedef struct
{
    INT   iIndex;            ///< \~chinese ģʽ������ \~english Mode index number
    char  acDescription[32]; ///< \~chinese ��ģʽ��������Ϣ \~english Description information of this mode
} tSdkTrigger;

/// \~chinese ����ְ���С����(���ĳЩ���������Ч)
/// \~english Transmission packet size description (valid for some web cameras)
typedef struct
{
    INT  iIndex;              ///< \~chinese �ְ���С������ \~english Packet size index number
    char acDescription[32];   ///< \~chinese ��Ӧ��������Ϣ \~english Corresponding description information
    UINT iPackSize;			  ///< \~chinese ����С \~english Packet size
} tSdkPackLength;

/// \~chinese Ԥ���LUT������
/// \~english Preset LUT Table Description
typedef struct
{
    INT  iIndex;                ///< \~chinese ���� \~english index
    char acDescription[32];     ///< \~chinese ������Ϣ \~english description
} tSdkPresetLut;

/// \~chinese AE�㷨����
/// \~english AE algorithm description
typedef struct
{
    INT  iIndex;                ///< \~chinese ���� \~english index
    char acDescription[32];     ///< \~chinese ������Ϣ \~english description
} tSdkAeAlgorithm;

/// \~chinese RAWתRGB�㷨����
/// \~english RAW to RGB algorithm description
typedef struct
{
    INT  iIndex;                ///< \~chinese ���� \~english index
    char acDescription[32];     ///< \~chinese ������Ϣ \~english description
} tSdkBayerDecodeAlgorithm;

/// \~chinese ֡ͳ����Ϣ
/// \~english Frame statistics
typedef struct
{
    INT iTotal;           ///< \~chinese ��ǰ�ɼ�����֡������������֡�� \~english The current total number of frames collected (including error frames)
    INT iCapture;       ///< \~chinese ��ǰ�ɼ�����Ч֡������ \~english The number of valid frames currently collected
    INT iLost;          ///< \~chinese ��ǰ��֡������ \~english Current number of dropped frames
} tSdkFrameStatistic;

/// \~chinese ��������ͼ�����ݸ�ʽ
/// \~english Camera output image data format
typedef struct
{
    INT     iIndex;             ///< \~chinese ��ʽ������ \~english Format type number
    char    acDescription[32];  ///< \~chinese ������Ϣ \~english description
    UINT    iMediaType;         ///< \~chinese ��Ӧ��ͼ���ʽ���룬��CAMERA_MEDIA_TYPE_BAYGR8�� \~english Corresponding image format encoding, such as CAMERA_MEDIA_TYPE_BAYGR8.
} tSdkMediaType;

/// \~chinese ٤����趨��Χ
/// \~english Gamma setting range
typedef struct
{
    INT iMin;       ///< \~chinese ��Сֵ \~english min
    INT iMax;       ///< \~chinese ���ֵ \~english max
} tGammaRange;

/// \~chinese �Աȶȵ��趨��Χ
/// \~english Contrast setting range
typedef struct
{
    INT iMin;   ///< \~chinese ��Сֵ \~english min
    INT iMax;   ///< \~chinese ���ֵ \~english max
} tContrastRange;

/// \~chinese RGB��ͨ������������趨��Χ
/// \~english RGB three channel digital gain setting range
typedef struct
{
    INT iRGainMin;    ///< \~chinese ��ɫ�������Сֵ \~english Red gain minimum
    INT iRGainMax;    ///< \~chinese ��ɫ��������ֵ \~english Red gain maximum
    INT iGGainMin;    ///< \~chinese ��ɫ�������Сֵ \~english Green gain minimum
    INT iGGainMax;    ///< \~chinese ��ɫ��������ֵ \~english Green gain maximum
    INT iBGainMin;    ///< \~chinese ��ɫ�������Сֵ \~english Blue gain minimum
    INT iBGainMax;    ///< \~chinese ��ɫ��������ֵ \~english blue gain maximum
} tRgbGainRange;

/// \~chinese ���Ͷ��趨�ķ�Χ
/// \~english Saturation setting range
typedef struct
{
    INT iMin;   ///< \~chinese ��Сֵ \~english min
    INT iMax;   ///< \~chinese ���ֵ \~english max
} tSaturationRange;

/// \~chinese �񻯵��趨��Χ
/// \~english Sharpening setting range
typedef struct
{
    INT iMin;   ///< \~chinese ��Сֵ \~english min
    INT iMax;   ///< \~chinese ���ֵ \~english max
} tSharpnessRange;

/// \~chinese ISPģ���ʹ����Ϣ
/// \~english ISP module enable information
typedef struct
{
    BOOL bMonoSensor;       ///< \~chinese ��ʾ���ͺ�����Ƿ�Ϊ�ڰ����,����Ǻڰ����������ɫ��صĹ��ܶ��޷����� \~english Indicates whether this model is a black-and-white camera. If it is a black-and-white camera, color-related functions cannot be adjusted.
    BOOL bWbOnce;           ///< \~chinese ��ʾ���ͺ�����Ƿ�֧���ֶ���ƽ�⹦�� \~english Indicates whether this model camera supports manual white balance function
    BOOL bAutoWb;           ///< \~chinese ��ʾ���ͺ�����Ƿ�֧���Զ���ƽ�⹦�� \~english Indicates whether this model camera supports automatic white balance function
    BOOL bAutoExposure;     ///< \~chinese ��ʾ���ͺ�����Ƿ�֧���Զ��ع⹦�� \~english Indicates whether this model camera supports auto exposure function
    BOOL bManualExposure;   ///< \~chinese ��ʾ���ͺ�����Ƿ�֧���ֶ��ع⹦�� \~english Indicates whether this model camera supports manual exposure function
    BOOL bAntiFlick;        ///< \~chinese ��ʾ���ͺ�����Ƿ�֧�ֿ�Ƶ������ \~english Indicates whether this model camera supports anti-strobe function
    BOOL bDeviceIsp;        ///< \~chinese ��ʾ���ͺ�����Ƿ�֧��Ӳ��ISP���� \~english Indicates whether this model camera supports hardware ISP function
    BOOL bForceUseDeviceIsp;///< \~chinese bDeviceIsp��bForceUseDeviceIspͬʱΪTRUEʱ����ʾǿ��ֻ��Ӳ��ISP������ȡ���� \~english When both bDeviceIsp and bForceUseDeviceIsp are TRUE, this means that only the hardware ISP is forced and cannot be cancelled.
    BOOL bZoomHD;           ///< \~chinese ���Ӳ���Ƿ�֧��ͼ���������(ֻ������С)�� \~english Whether the camera hardware supports image scaling output (can only be reduced).
} tSdkIspCapacity;

/// \~chinese �������ϵ��豸������Ϣ����Щ��Ϣ�������ڶ�̬����UI
/// \note ����@link #CameraGetCapability @endlink��ȡ���ṹ
/// \~english Define integrated device description information that can be used to dynamically build UI
/// \note call @link #CameraGetCapability @endlink to get this structure
typedef struct
{

    tSdkTrigger   *pTriggerDesc;          ///< \~chinese ����ģʽ \~english trigger mode
    INT           iTriggerDesc;           ///< \~chinese ����ģʽ�ĸ�������pTriggerDesc����Ĵ�С \~english The number of trigger modes, that is, the size of the pTriggerDesc array

    tSdkImageResolution   *pImageSizeDesc;///< \~chinese Ԥ��ֱ��� \~english Preset resolution
    INT                   iImageSizeDesc; ///< \~chinese Ԥ��ֱ��ʵĸ�������pImageSizeDesc����Ĵ�С \~english The number of preset resolutions, that is, the size of the pImageSizeDesc array

    tSdkColorTemperatureDes *pClrTempDesc;///< \~chinese Ԥ��ɫ�£����ڰ�ƽ�� \~english Preset color temperature for white balance
    INT                     iClrTempDesc; ///< \~chinese Ԥ��ɫ�¸��� \~english The number of preset color temperatures

    tSdkMediaType     *pMediaTypeDesc;    ///< \~chinese ������ͼ���ʽ \~english Camera output image format
    INT               iMediaTypdeDesc;    ///< \~chinese ������ͼ���ʽ�������������pMediaTypeDesc����Ĵ�С�� \~english The number of types of camera output image formats, that is, the size of the pMediaTypeDesc array.

    tSdkFrameSpeed    *pFrameSpeedDesc;   ///< \~chinese �ɵ���֡�����ͣ���Ӧ��������ͨ ���� �ͳ��������ٶ����� \~english Adjustable frame rate type, normal high speed and super three speed settings on the corresponding interface
    INT               iFrameSpeedDesc;    ///< \~chinese �ɵ���֡�����͵ĸ�������pFrameSpeedDesc����Ĵ�С�� \~english The number of frame rate types that can be adjusted, that is, the size of the pFrameSpeedDesc array.

    tSdkPackLength    *pPackLenDesc;      ///< \~chinese ��������ȣ�һ�����������豸 \~english Transmission packet length, generally used for network equipment
    INT               iPackLenDesc;       ///< \~chinese �ɹ�ѡ��Ĵ���ְ����ȵĸ�������pPackLenDesc����Ĵ�С��  \~english The number of transmission packetization lengths available for selection, which is the size of the pPackLenDesc array.

    INT           iOutputIoCounts;        ///< \~chinese �ɱ�����IO�ĸ��� \~english Number of programmable output IOs
    INT           iInputIoCounts;         ///< \~chinese �ɱ������IO�ĸ��� \~english Number of programmable input IOs

    tSdkPresetLut  *pPresetLutDesc;       ///< \~chinese ���Ԥ���LUT�� \~english Camera preset LUT table
    INT            iPresetLut;            ///< \~chinese ���Ԥ���LUT��ĸ�������pPresetLutDesc����Ĵ�С \~english The number of LUT tables preset by the camera, that is, the size of the pPresetLutDesc array

    INT           iUserDataMaxLen;        ///< \~chinese ָʾ����������ڱ����û�����������󳤶ȡ�Ϊ0��ʾ�ޡ� \~english Indicates the maximum length in the camera used to save the user data area. 0 means no.
    BOOL          bParamInDevice;         ///< \~chinese ָʾ���豸�Ƿ�֧�ִ��豸�ж�д�����顣1Ϊ֧�֣�0��֧�֡� \~english Indicates whether the device supports reading and writing parameter groups from the device. 1 is supported, 0 is not supported.

    tSdkAeAlgorithm   *pAeAlmSwDesc;      ///< \~chinese ����Զ��ع��㷨���� \~english Software auto exposure algorithm description
    int                iAeAlmSwDesc;      ///< \~chinese ����Զ��ع��㷨���� \~english Software automatic exposure algorithm number

    tSdkAeAlgorithm    *pAeAlmHdDesc;     ///< \~chinese Ӳ���Զ��ع��㷨������ΪNULL��ʾ��֧��Ӳ���Զ��ع� \~english Hardware auto exposure algorithm description, NULL means hardware auto exposure is not supported
    int                iAeAlmHdDesc;      ///< \~chinese Ӳ���Զ��ع��㷨������Ϊ0��ʾ��֧��Ӳ���Զ��ع� \~english Number of hardware auto exposure algorithms, 0 means hardware auto exposure is not supported

    tSdkBayerDecodeAlgorithm   *pBayerDecAlmSwDesc; ///< \~chinese ���Bayerת��ΪRGB���ݵ��㷨���� \~english Algorithm Description of Software Bayer Conversion to RGB Data
    int                        iBayerDecAlmSwDesc;  ///< \~chinese ���Bayerת��ΪRGB���ݵ��㷨���� \~english The number of algorithms that Bayer converts to RGB data

    tSdkBayerDecodeAlgorithm   *pBayerDecAlmHdDesc; ///< \~chinese Ӳ��Bayerת��ΪRGB���ݵ��㷨������ΪNULL��ʾ��֧�� \~english Hardware Bayer converts to RGB data algorithm description, is not supported for NULL representation
    int                        iBayerDecAlmHdDesc;  ///< \~chinese Ӳ��Bayerת��ΪRGB���ݵ��㷨������Ϊ0��ʾ��֧�� \~english The number of algorithms that hardware Bayer converts to RGB data, 0 means not supported

    /* ͼ������ĵ��ڷ�Χ����,���ڶ�̬����UI*/
    tSdkExpose            sExposeDesc;      ///< \~chinese �ع�ķ�Χֵ \~english Exposure range value
    tSdkResolutionRange   sResolutionRange; ///< \~chinese �ֱ��ʷ�Χ���� \~english Resolution range description
    tRgbGainRange         sRgbGainRange;    ///< \~chinese ͼ���������淶Χ���� \~english Image digital gain range description
    tSaturationRange      sSaturationRange; ///< \~chinese ���Ͷȷ�Χ���� \~english Saturation range description
    tGammaRange           sGammaRange;      ///< \~chinese ٤��Χ���� \~english Gamma range description
    tContrastRange        sContrastRange;   ///< \~chinese �Աȶȷ�Χ���� \~english Contrast range description
    tSharpnessRange       sSharpnessRange;  ///< \~chinese �񻯷�Χ���� \~english Sharpening range description
    tSdkIspCapacity       sIspCapacity;     ///< \~chinese ISP�������� \~english ISP capability description


} tSdkCameraCapbility;


/// \~chinese ͼ��֡ͷ��Ϣ
/// \~english Image frame header information
typedef struct
{
    UINT    uiMediaType;    ///< \~chinese ͼ���ʽ \~english Image Format
    UINT    uBytes;         ///< \~chinese ͼ�������ֽ��� \~english Total bytes
    INT     iWidth;         ///< \~chinese ͼ��Ŀ�ȣ�����ͼ�������󣬸ñ������ܱ���̬�޸ģ���ָʾ������ͼ��ߴ� \~english The width of the image, after calling the image processing function, the variable may be dynamically modified to indicate the image size after processing
    INT     iHeight;        ///< \~chinese ͼ��ĸ߶ȣ�����ͼ�������󣬸ñ������ܱ���̬�޸ģ���ָʾ������ͼ��ߴ� \~english The height of the image, after calling the image processing function, the variable may be dynamically modified to indicate the image size after processing
    INT     iWidthZoomSw;   ///< \~chinese ������ŵĿ��,����Ҫ��������ü���ͼ�񣬴˱�������Ϊ0. \~english The width of the software zoom, which does not require software cropping. This variable is set to 0.
    INT     iHeightZoomSw;  ///< \~chinese ������ŵĸ߶�,����Ҫ��������ü���ͼ�񣬴˱�������Ϊ0. \~english Software zoom height, no software cropped image is required. This variable is set to 0.
    BOOL    bIsTrigger;     ///< \~chinese ָʾ�Ƿ�Ϊ����֡ \~english is trigger
    UINT    uiTimeStamp;    ///< \~chinese ��֡�Ĳɼ�ʱ�䣬��λ0.1���� \~english The frame acquisition time, in units of 0.1 milliseconds
    UINT    uiExpTime;      ///< \~chinese ��ǰͼ����ع�ֵ����λΪ΢��us \~english Exposure of the current image in microseconds us
    float   fAnalogGain;    ///< \~chinese ��ǰͼ���ģ�����汶�� \~english The current image's analog gain multiplier
    INT     iGamma;         ///< \~chinese ��֡ͼ���٤���趨ֵ������LUTģʽΪ��̬��������ʱ��Ч������ģʽ��Ϊ-1 \~english The gamma setting value of the frame image is valid only when the LUT mode is a dynamic parameter generation, and is -1 in other modes.
    INT     iContrast;      ///< \~chinese ��֡ͼ��ĶԱȶ��趨ֵ������LUTģʽΪ��̬��������ʱ��Ч������ģʽ��Ϊ-1 \~english The contrast setting value of the frame image is only valid when the LUT mode is generated by the dynamic parameter, and is -1 in other modes.
    INT     iSaturation;    ///< \~chinese ��֡ͼ��ı��Ͷ��趨ֵ�����ںڰ���������壬Ϊ0 \~english The saturation value of the frame image, which is meaningless for a black and white camera, is 0
    float   fRgain;         ///< \~chinese ��֡ͼ����ĺ�ɫ�������汶�������ںڰ���������壬Ϊ1 \~english The red digital gain multiple of this frame image processing is meaningless for a black and white camera and is 1
    float   fGgain;         ///< \~chinese ��֡ͼ�������ɫ�������汶�������ںڰ���������壬Ϊ1 \~english The green digital gain multiplier for this frame image processing, meaning no significance for black and white cameras, is 1
    float   fBgain;         ///< \~chinese ��֡ͼ�������ɫ�������汶�������ںڰ���������壬Ϊ1 \~english The blue digital gain multiplier for this frame image processing, meaning no significance for black and white cameras, is 1
}tSdkFrameHead;

/// \~chinese ͼ��֡����
/// \~english Image frame description
typedef struct sCameraFrame
{
    tSdkFrameHead   head;     ///< \~chinese ֡ͷ \~english Frame Head
    BYTE *          pBuffer;  ///< \~chinese ������ \~english Image Data
}tSdkFrame;

/// \~chinese ֡�¼�
/// \~english Frame Event
typedef struct tSdkFrameEvent_
{
    UINT 	uType;			///< \~chinese �¼�����(1:֡��ʼ   2:֡����) \~english Event type (1:frame start   2:frame end)
    UINT	uStatus;		///< \~chinese ״̬(0:�ɹ�  ��0:����) \~english Status (0:success, non-zero:error)
    UINT 	uFrameID;		///< \~chinese ֡ID \~english Frame ID
    UINT	uWidth;			///< \~chinese ��� \~english Width
    UINT	uHeight;		///< \~chinese �߶� \~english Height
    UINT	uPixelFormat;	///< \~chinese ͼ���ʽ \~english Image Format
    UINT	TimeStampL;		///< \~chinese ʱ�����32λ \~english Lower 32 bits of timestamp
    UINT	TimeStampH;		///< \~chinese ʱ�����32λ \~english High 32 bits of timestamp
}tSdkFrameEvent;

/// @ingroup API_GRAB_CB
/// \~chinese ͼ�񲶻�Ļص���������
/// \~english Callback function definition for image capture
typedef void (WINAPI* CAMERA_SNAP_PROC)(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext);

/// @ingroup API_SETTINGS_PAGE
/// \~chinese �������ҳ�����Ϣ�ص���������
/// \~english camera configuration page message callback function definition
typedef void (WINAPI* CAMERA_PAGE_MSG_PROC)(CameraHandle hCamera,UINT MSG,UINT uParam,PVOID pContext);

/// @ingroup API_RECONNECT
/// \~chinese �������״̬�ص�
/// \param [in] hCamera ������
/// \param [in] MSG ��Ϣ��0: ������ӶϿ�    1: ������ӻָ�
/// \param [in] uParam ������Ϣ
/// \param [in] pContext �û�����
/// \return ��
/// \note USB���uParamȡֵ��
/// \note 		δ����
/// \note �������uParamȡֵ��
/// \note		��MSG=0ʱ��δ����
/// \note		��MSG=1ʱ��
/// \note			0���ϴε���ԭ������ͨѶʧ��
/// \note			1���ϴε���ԭ���������
/// \~english Camera connection status callback
/// \param [in] hCamera Camera handle
/// \param [in] MSG message, 0: Camera disconnected 1: Camera connection restored
/// \param [in] uParam Additional Information
/// \param [in] pContext user data
/// \return None
/// \note USB camera uParam value:
/// \note       Undefined
/// \note network camera uParam value:
/// \note       When MSG=0: Undefined
/// \note       When MSG=1:
/// \note           0: The last dropped reason, network communication failed
/// \note           1: The last dropped reason, the camera lost power
typedef void (WINAPI* CAMERA_CONNECTION_STATUS_CALLBACK)(CameraHandle hCamera,UINT MSG,UINT uParam,PVOID pContext);

/// @ingroup API_ADVANCE
/// \~chinese ֡�¼��ص���������
/// \~english Callback function definition for frame event
typedef void (WINAPI* CAMERA_FRAME_EVENT_CALLBACK)(CameraHandle hCamera, tSdkFrameEvent* pEvent, PVOID pContext);


//////////////////////////////////////////////////////////////////////////
// Grabber ���

/// \~chinese Grabberͳ����Ϣ
/// \~english Grabber statistics
typedef struct
{
    int Width;			///< \~chinese ֡��� \~english Frame image width
    int Height;			///< \~chinese ֡�߶� \~english Frame image height
    int Disp;			///< \~chinese ��ʾ֡���� \~english Display frame number
    int Capture;		///< \~chinese �ɼ�����Ч֡������ \~english The number of valid frames collected
    int Lost;			///< \~chinese ��֡������ \~english The number of dropped frames
    int Error;			///< \~chinese ��֡������ \~english The number of error frames
    float DispFps;		///< \~chinese ��ʾ֡�� \~english Display frame rate
    float CapFps;		///< \~chinese ����֡�� \~english Capture frame rate
}tSdkGrabberStat;

/// @ingroup GRABBER_CB
/// \~chinese ͼ�񲶻�Ļص���������
/// \~english Callback function definition for image capture
typedef void (__stdcall *pfnCameraGrabberFrameCallback)(
        void* Grabber,
        BYTE *pFrameBuffer,
        tSdkFrameHead* pFrameHead,
        void* Context);

/// @ingroup GRABBER_CB
/// \~chinese ֡������������
/// \param [in] Grabber
/// \param [in] Phase ͼ����׶�
/// \param [in] pFrameBuffer ֡����
/// \param [in] pFrameHead ֡ͷ
/// \param [in] Context �û�����
/// \return 0:Grabber���ᶪ����֡��������Դ�֡�����к�������׶�   1:������һ�׶δ���
/// \note ÿ��Grabber����һ֡ͼ��ʱ�����3���׶������ε���FrameListener
/// \note �׶�0: RAW���ݴ���pFrameBuffer=Raw����
/// \note �׶�1: ��ͼǰ����pFrameBuffer=RGB����
/// \note �׶�2: ��ʾǰ����pFrameBuffer=RGB����
/// \note �ر�ģ���������ߺ�˻ص�Ҳ�ᱻ���ã���ʱPhase=-1��pFrameBuffer=NULL,pFrameHead=NULL��
/// \~english Frame listening function definition
/// \param [in] Grabber
/// \param [in] Phase image processing phase
/// \param [in] pFrameBuffer frame data
/// \param [in] pFrameHead Frame Header
/// \param [in] Context user data
/// \return 0: Grabber will discard this frame and end all subsequent processing stages for this frame     1: Continue to the next stage of processing
/// \note Whenever Grabber captures a frame of image, it will call FrameListener in turn in 3 stages.
/// \note Phase 0: RAW data processing, pFrameBuffer= Raw data
/// \note Phase 1: Screenshot pre-processing, pFrameBuffer=RGB data
/// \note Phase 2: Display preprocessing, pFrameBuffer=RGB data
/// \note In particular, this callback will be called when the camera is disconnected. At this time, Phase=-1, pFrameBuffer=NULL, and pFrameHead=NULL.
typedef int (__stdcall *pfnCameraGrabberFrameListener)(
        void* Grabber,
        int Phase,
        BYTE *pFrameBuffer,
        tSdkFrameHead* pFrameHead,
        void* Context);

/// @ingroup GRABBER_SNAPSHOT
/// \~chinese �첽ץͼ�Ļص���������
/// \warning Image��Ҫ���� @link CameraImage_Destroy @endlink �ͷ�
/// \~english Asynchronous snapshot callback function definition
/// \warning Image needs to call @link CameraImage_Destroy @endlink to release
typedef void (__stdcall *pfnCameraGrabberSaveImageComplete)(
        void* Grabber,
        void* Image,	// ��Ҫ����CameraImage_Destroy�ͷ�
        CameraSdkStatus Status,
        void* Context
        );


/// @ingroup MV_MACRO_TYPE 
/// @{
//----------------------------IMAGE FORMAT DEFINE------------------------------------
//----------------------------ͼ���ʽ����-------------------------------------------
#define CAMERA_MEDIA_TYPE_MONO                           0x01000000
#define CAMERA_MEDIA_TYPE_RGB                            0x02000000 
#define CAMERA_MEDIA_TYPE_COLOR                          0x02000000
#define CAMERA_MEDIA_TYPE_CUSTOM                         0x80000000
#define CAMERA_MEDIA_TYPE_COLOR_MASK                     0xFF000000
#define CAMERA_MEDIA_TYPE_OCCUPY1BIT                     0x00010000
#define CAMERA_MEDIA_TYPE_OCCUPY2BIT                     0x00020000
#define CAMERA_MEDIA_TYPE_OCCUPY4BIT                     0x00040000
#define CAMERA_MEDIA_TYPE_OCCUPY8BIT                     0x00080000
#define CAMERA_MEDIA_TYPE_OCCUPY10BIT                    0x000A0000
#define CAMERA_MEDIA_TYPE_OCCUPY12BIT                    0x000C0000
#define CAMERA_MEDIA_TYPE_OCCUPY16BIT                    0x00100000
#define CAMERA_MEDIA_TYPE_OCCUPY24BIT                    0x00180000
#define CAMERA_MEDIA_TYPE_OCCUPY32BIT                    0x00200000
#define CAMERA_MEDIA_TYPE_OCCUPY36BIT                    0x00240000
#define CAMERA_MEDIA_TYPE_OCCUPY48BIT                    0x00300000
#define CAMERA_MEDIA_TYPE_OCCUPY64BIT					 0x00400000

#define CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_MASK      0x00FF0000
#define CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_SHIFT     16

#define CAMERA_MEDIA_TYPE_PIXEL_SIZE(type)				 (((type) & CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_MASK) >> CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_SHIFT)


#define CAMERA_MEDIA_TYPE_ID_MASK                        0x0000FFFF
#define CAMERA_MEDIA_TYPE_COUNT                          0x46 

/*mono*/
#define CAMERA_MEDIA_TYPE_MONO1P             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY1BIT | 0x0037)
#define CAMERA_MEDIA_TYPE_MONO2P             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY2BIT | 0x0038)
#define CAMERA_MEDIA_TYPE_MONO4P             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY4BIT | 0x0039)
#define CAMERA_MEDIA_TYPE_MONO8              (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0001)
#define CAMERA_MEDIA_TYPE_MONO8S             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0002)
#define CAMERA_MEDIA_TYPE_MONO10             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0003)
#define CAMERA_MEDIA_TYPE_MONO10_PACKED      (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0004)
#define CAMERA_MEDIA_TYPE_MONO12             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0005)
#define CAMERA_MEDIA_TYPE_MONO12_PACKED      (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0006)
#define CAMERA_MEDIA_TYPE_MONO14             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0025)
#define CAMERA_MEDIA_TYPE_MONO16             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0007)

/*Bayer */
#define CAMERA_MEDIA_TYPE_BAYGR8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0008)
#define CAMERA_MEDIA_TYPE_BAYRG8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0009)
#define CAMERA_MEDIA_TYPE_BAYGB8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x000A)
#define CAMERA_MEDIA_TYPE_BAYBG8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x000B)

#define CAMERA_MEDIA_TYPE_BAYGR10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0026)
#define CAMERA_MEDIA_TYPE_BAYRG10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0027)
#define CAMERA_MEDIA_TYPE_BAYGB10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0028)
#define CAMERA_MEDIA_TYPE_BAYBG10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0029)


#define CAMERA_MEDIA_TYPE_BAYGR10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000C)
#define CAMERA_MEDIA_TYPE_BAYRG10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000D)
#define CAMERA_MEDIA_TYPE_BAYGB10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000E)
#define CAMERA_MEDIA_TYPE_BAYBG10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000F)

#define CAMERA_MEDIA_TYPE_BAYGR12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0010)
#define CAMERA_MEDIA_TYPE_BAYRG12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0011)
#define CAMERA_MEDIA_TYPE_BAYGB12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0012)
#define CAMERA_MEDIA_TYPE_BAYBG12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0013)


#define CAMERA_MEDIA_TYPE_BAYGR10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0026)
#define CAMERA_MEDIA_TYPE_BAYRG10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0027)
#define CAMERA_MEDIA_TYPE_BAYGB10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0028)
#define CAMERA_MEDIA_TYPE_BAYBG10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0029)

#define CAMERA_MEDIA_TYPE_BAYGR12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002A)
#define CAMERA_MEDIA_TYPE_BAYRG12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002B)
#define CAMERA_MEDIA_TYPE_BAYGB12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002C)
#define CAMERA_MEDIA_TYPE_BAYBG12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002D)

#define CAMERA_MEDIA_TYPE_BAYGR16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x002E)
#define CAMERA_MEDIA_TYPE_BAYRG16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x002F)
#define CAMERA_MEDIA_TYPE_BAYGB16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0030)
#define CAMERA_MEDIA_TYPE_BAYBG16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0031)

/*RGB */
#define CAMERA_MEDIA_TYPE_RGB8               (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0014)
#define CAMERA_MEDIA_TYPE_BGR8               (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0015)
#define CAMERA_MEDIA_TYPE_RGBA8              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x0016)
#define CAMERA_MEDIA_TYPE_BGRA8              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x0017)
#define CAMERA_MEDIA_TYPE_RGB10              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0018)
#define CAMERA_MEDIA_TYPE_BGR10              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0019)
#define CAMERA_MEDIA_TYPE_RGB12              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x001A)
#define CAMERA_MEDIA_TYPE_BGR12              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x001B)
#define CAMERA_MEDIA_TYPE_RGB16              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0033)
#define CAMERA_MEDIA_TYPE_BGR16              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x004B)
#define CAMERA_MEDIA_TYPE_RGBA16             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY64BIT | 0x0064)
#define CAMERA_MEDIA_TYPE_BGRA16             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY64BIT | 0x0051)
#define CAMERA_MEDIA_TYPE_RGB10V1_PACKED     (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x001C)
#define CAMERA_MEDIA_TYPE_RGB10P32           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x001D)
#define CAMERA_MEDIA_TYPE_RGB12V1_PACKED     (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY36BIT | 0X0034)
#define CAMERA_MEDIA_TYPE_RGB565P            (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0035)
#define CAMERA_MEDIA_TYPE_BGR565P            (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0X0036)

/*YUV and YCbCr*/
#define CAMERA_MEDIA_TYPE_YUV411_8_UYYVYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x001E)
#define CAMERA_MEDIA_TYPE_YUV422_8_UYVY      (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x001F)
#define CAMERA_MEDIA_TYPE_YUV422_8           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0032)
#define CAMERA_MEDIA_TYPE_YUV8_UYV           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0020)
#define CAMERA_MEDIA_TYPE_YCBCR8_CBYCR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x003A)
//CAMERA_MEDIA_TYPE_YCBCR422_8 : YYYYCbCrCbCr
#define CAMERA_MEDIA_TYPE_YCBCR422_8             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x003B)
#define CAMERA_MEDIA_TYPE_YCBCR422_8_CBYCRY      (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0043)
#define CAMERA_MEDIA_TYPE_YCBCR411_8_CBYYCRYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x003C)
#define CAMERA_MEDIA_TYPE_YCBCR601_8_CBYCR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x003D)
#define CAMERA_MEDIA_TYPE_YCBCR601_422_8         (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x003E)
#define CAMERA_MEDIA_TYPE_YCBCR601_422_8_CBYCRY  (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0044)
#define CAMERA_MEDIA_TYPE_YCBCR601_411_8_CBYYCRYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x003F)
#define CAMERA_MEDIA_TYPE_YCBCR709_8_CBYCR           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0040)
#define CAMERA_MEDIA_TYPE_YCBCR709_422_8             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0041)
#define CAMERA_MEDIA_TYPE_YCBCR709_422_8_CBYCRY      (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0045)
#define CAMERA_MEDIA_TYPE_YCBCR709_411_8_CBYYCRYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0042)

/*RGB Planar */
#define CAMERA_MEDIA_TYPE_RGB8_PLANAR        (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0021)
#define CAMERA_MEDIA_TYPE_RGB10_PLANAR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0022)
#define CAMERA_MEDIA_TYPE_RGB12_PLANAR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0023)
#define CAMERA_MEDIA_TYPE_RGB16_PLANAR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0024)

/*MindVision 12bit packed bayer*/
#define CAMERA_MEDIA_TYPE_BAYGR12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0060)
#define CAMERA_MEDIA_TYPE_BAYRG12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0061)
#define CAMERA_MEDIA_TYPE_BAYGB12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0062)
#define CAMERA_MEDIA_TYPE_BAYBG12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0063)

/*MindVision 12bit packed monochome*/
#define CAMERA_MEDIA_TYPE_MONO12_PACKED_MV		(CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0064)
#define CAMERA_MEDIA_TYPE_YUV420P_MV			(CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0065)

/*planar YUV 4:2:0, 12bpp, 1 plane for Y and 1 plane for the UV components, which are interleaved (first byte V and the following byte U)*/
#define CAMERA_MEDIA_TYPE_YUV_NV21_MV			(CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0066)

/* H264 H265 */
#define CAMERA_MEDIA_TYPE_H264_MV				(CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0067)
#define CAMERA_MEDIA_TYPE_H265_MV				(CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0068)

/* JPEG */
#define CAMERA_MEDIA_TYPE_JPEG_MV				(CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0069)

/// @}

#endif
