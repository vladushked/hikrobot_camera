#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include <ros/ros.h>

namespace camera
{
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4 * 3648 * 5472)
    //********** frame ************************************/
    cv::Mat frame;
    //********** capture time и frame index ******************************/
    ros::Time capture_time;
    unsigned long frame_index = 0;
    //********** frame_empty ******************************/
    bool frame_empty = 0;
    //********** mutex ************************************/
    pthread_mutex_t mutex;
    //********** CameraProperties config ************************************/
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_GAIN,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线

    };
    float resize_scale;

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class************************************ //
    //^ *********************************************************************************** //
    class Camera
    {
    public:
        //********** 构造函数  ****************************/
        Camera(ros::NodeHandle &node, std::string serial_number);
        //********** 析构函数  ****************************/
        ~Camera();
        //********** 原始信息转换线程 **********************/
        static void *HKWorkThread(void *p_handle);

        //********** 输出摄像头信息 ***********************/
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        //********** 读图10个相机的原始图像 ********************************/
        void ReadImg(cv::Mat &image, ros::Time &capture_time, unsigned long &frame_index);

        bool FrameEmpty();

    private:
        //********** handle ******************************/
        void *handle;
        //********** nThreadID ******************************/
        pthread_t nThreadID;
        //********** yaml config ******************************/
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        float FrameRate;
        int BurstFrameCount;
        float ExposureTime;
        bool GammaEnable;
        float Gamma;
        int GainAuto;
        float Gain;
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;
        int SensorShutterMode;
    };
    //^ *********************************************************************************** //

    //^ ********************************** Camera constructor************************************ //
    Camera::Camera(ros::NodeHandle &node, std::string serial_number)
    {
        handle = NULL;

        printf("Desired Serial Number: %s\n", serial_number.c_str());

        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        node.param("width", width, 5472);
        node.param("height", height, 3648);
        node.param("resize_scale", resize_scale, 0.125f);
        node.param("FrameRateEnable", FrameRateEnable, true);
        node.param("FrameRate", FrameRate, (float)10.0);
        node.param("BurstFrameCount", BurstFrameCount, 10); // 一次触发采集的次数
        node.param("ExposureTime", ExposureTime, 1000.0f);
        node.param("GammaEnable", GammaEnable, false);
        node.param("Gamma", Gamma, (float)0.7);
        node.param("GainAuto", GainAuto, 0);
        node.param("Gain", Gain, (float)0.0);
        node.param("SaturationEnable", SaturationEnable, true);
        node.param("Saturation", Saturation, 128);
        node.param("Offset_x", Offset_x, 0);
        node.param("Offset_y", Offset_y, 0);
        node.param("TriggerMode", TriggerMode, 1);
        node.param("TriggerSource", TriggerSource, 2);
        node.param("LineSelector", LineSelector, 2);
        node.param("SensorShutterMode", SensorShutterMode, 1);

        printf("Resize scale: %f\n", resize_scale);

        //********** 枚举设备 ********************************/
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // Перебираем все найденные устройства и ищем камеру с заданным серийным номером
        MV_CC_DEVICE_INFO *pSelectedDevice = NULL;

        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
                // Для USB-камер используем серийный номер
                if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
                {
                    printf("Device serial num: %s, provided num: %s\n", pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber, serial_number.c_str());
                    if (strcmp(reinterpret_cast<const char *>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), serial_number.c_str()) == 0)
                    {
                        pSelectedDevice = pDeviceInfo;
                        break;
                    }
                }
                // Если необходимо, можно добавить аналогичное сравнение для GigE камер,
                // если у них имеется поле серийного номера.
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        if (pSelectedDevice == NULL)
        {
            printf("No device found with serial number: %s\n", serial_number.c_str());
            exit(-1);
        }

        //********** 选择设备并创建句柄 *************************/

        nRet = MV_CC_CreateHandle(&handle, pSelectedDevice);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        //********** frame **********/

        nRet = MV_CC_OpenDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", FrameRateEnable);

        if (MV_OK == nRet)
        {
            printf("set AcquisitionFrameRateEnable OK! value=%d\n", FrameRateEnable);
        }
        else
        {
            printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", FrameRate);

        if (MV_OK == nRet)
        {
            printf("set AcquisitionFrameRate OK! value=%f\n", FrameRate);
        }
        else
        {
            printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetExposureTime(handle, ExposureTime);
        if (MV_OK == nRet)
        {
            printf("set ExposureTime OK! value=%f\n", ExposureTime);
        }
        else
        {
            printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetIntValue(handle, "Height", height);
        if (MV_OK == nRet)
        {
            printf("set Height OK! value=%d\n", height);
        }
        else
        {
            printf("Set Height Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetIntValue(handle, "Width", width);
        if (MV_OK == nRet)
        {
            printf("set Width OK! value=%d\n", width);
        }
        else
        {
            printf("Set Width Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto); // 亮度 越大越亮
        if (MV_OK == nRet)
        {
            printf("set GainAuto OK! value=%d\n", GainAuto);
        }
        else
        {
            printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
        }
        nRet = MV_CC_SetGain(handle, Gain);
        if (MV_OK == nRet)
        {
            printf("set Gain OK! value=%f\n", Gain);
        }
        else
        {
            printf("Set Gain Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetTriggerMode(handle, TriggerMode);
        if (MV_OK == nRet)
        {
            printf("set TriggerMode OK! value=%d\n", TriggerMode);
        }
        else
        {
            printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetEnumValue(handle, "SensorShutterMode", SensorShutterMode); // 饱和度 默认128 最大255
        if (MV_OK == nRet)
        {
            printf("set SensorShutterMode OK! value=%d\n", SensorShutterMode);
        }
        else
        {
            printf("Set SensorShutterMode Failed! nRet = [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);

        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n", 0.0);
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }

        //********** 图像格式 **********/
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014); // 目前 RGB

        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = RGB\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
        }
        MVCC_ENUMVALUE t = {0};
        //********** frame **********/

        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);

        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }
        // 开始取流
        //********** frame **********/

        nRet = MV_CC_StartGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // nRet = MV_CC_SetExposureTime(handle, ExposureTime);
        // if (MV_OK == nRet)
        // {
        //     printf("set ExposureTime OK! value=%f\n", ExposureTime);
        // }
        // else
        // {
        //     printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
        // }

        // 初始化互斥量
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        //********** frame **********/

        nRet = pthread_create(&nThreadID, NULL, HKWorkThread, handle);

        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            exit(-1);
        }
    }

    //^ ********************************** Camera constructor************************************ //
    Camera::~Camera()
    {
        int nRet;
        //********** frame **********/

        pthread_join(nThreadID, NULL);

        //********** frame **********/

        nRet = MV_CC_StopGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        //********** frame **********/

        nRet = MV_CC_CloseDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        // 销毁句柄
        //********** frame **********/

        nRet = MV_CC_DestroyHandle(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    //^ ********************************** PrintDeviceInfo ************************************ //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 // 当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); // 用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("chSerialNumber:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ ********************************** Camera constructor************************************ //
    void Camera::ReadImg(cv::Mat &image, ros::Time &capture_time, unsigned long &frame_index)
    {
        pthread_mutex_lock(&mutex);
        if (frame_empty)
        {
            image = cv::Mat();
        }
        else
        {
            image = camera::frame.clone();
            capture_time = camera::capture_time;
            frame_index = camera::frame_index;
            frame_empty = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    //^ ********************************** HKWorkThread1 ************************************ //
    void *Camera::HKWorkThread(void *p_handle)
    {
        double start;
        int nRet;
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0; // 空图帧数
        while (ros::ok())
        {
            start = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 10000);
            ros::Time current_time = ros::Time::now();
            if (nRet == MV_OK)
            {
                printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
            }
            else
            {
                // printf("%s: No data[%x]\n", fname.c_str(), nRet);
                continue;
            }
            image_empty_count = 0; // 空图帧数
            // 转换图像格式为BGR8

            stConvertParam.nWidth = 5472;                               // ch:图像宽 | en:image width
            stConvertParam.nHeight = 3648;                              // ch:图像高 | en:image height
            stConvertParam.pSrcData = m_pBufForDriver;                  // ch:输入数据缓存 | en:input data buffer
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           // ch:输入数据大小 | en:input data size
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; // ch:输出像素格式 | en:output pixel format                      //! 输出格式 RGB
            stConvertParam.pDstBuffer = m_pBufForSaveImage;             // ch:输出数据缓存 | en:output data buffer
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        // ch:输出缓存大小 | en:output buffer size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    // ch:输入像素格式 | en:input pixel format                       //! 输入格式 RGB
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);

            cv::Mat original = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone();
            cv::Mat resized;
            cv::resize(original, resized, cv::Size(), resize_scale, resize_scale);

            pthread_mutex_lock(&mutex);

            camera::frame = resized;
            camera::capture_time = current_time;
            camera::frame_index++;
            frame_empty = 0;

            pthread_mutex_unlock(&mutex);
            double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            //*************************************testing img********************************//
            // std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1 / time << std::endl;
            // imshow("HK vision",frame);
            // waitKey(1);
        }
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);
        return 0;
    }

} // namespace camera
#endif
