#ifndef CAMERA_MINDVISION_RUNNING_H
#define CAMERA_MINDVISION_RUNNING_H

#include "CameraApi.h"  //相机SDK的API头文件

#include "opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <stdio.h>

using namespace cv;

class MindvisionCamera
{
public:
    int  CameraParamInit(int iLanguageSel);  //初始化
    bool CameraParamSet();

    bool runCamera();  //运行相机
    bool Do();
    bool getImage(cv::Mat& src);  //获取图像

    bool SetExposureTime(double fExposureTime);        //设置曝光时间
    bool SetGain(int iRGain, int iGGain, int iBGain);  //设置图像增益
    bool SetGamma(int camera_gamma);                   //设置伽马
    bool SetSize();
    bool SetFrameSpeed();

    tSdkCameraDevInfo tCameraEnumList;
    int               iCameraCounts = 1;
    int               iStatus       = -1;

    int                 hCamera;      //相机的句柄
    tSdkCameraCapbility tCapability;  //设备描述信息
    int                 channel = 3;

    tSdkFrameHead  sFrameInfo;
    BYTE*          pbyBuffer;
    unsigned char* g_pRgbBuffer;  //处理后数据缓存区
    IplImage*      iplImage;

    cv::Mat             src_;
    BOOL                bAeState;  //相机曝光模式
    tSdkImageResolution tRes;
    int                 iFrameSpeed;

private:
    double exposure_time;  // 曝光时间
    int    camera_gamma;   //相机伽马
    int    iRGain;         // R gain
    int    iGGain;         // G gain
    int    iBGain;         // B Gain
};

#endif  // CAMERA_MINDVISION_RUNNING_H
