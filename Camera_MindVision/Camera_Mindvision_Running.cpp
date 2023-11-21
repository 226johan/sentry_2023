#include "Camera_MindVision/Camera_Mindvision_Running.h"

using namespace cv;

int MindvisionCamera::CameraParamInit(int iLanguageSel)
{

    CameraSdkInit(iLanguageSel);
    // read  the camera parameter from yaml file
    cv::FileStorage fs_read("/home/qingling/thread/sentry7/Params/CameraParams_MindVision.yml", cv::FileStorage::READ);


    fs_read["ExposureTime"] >> exposure_time;

    std::vector<int> camera_gain;
    fs_read["Gain"] >> camera_gain;
    iRGain = camera_gain.at(0);
    iGGain = camera_gain.at(1);
    iBGain = camera_gain.at(2);

    fs_read["FrameSpeed"] >> iFrameSpeed;

    memset(&tRes, 0, sizeof(tSdkImageResolution));
    std::vector<int> imagesize;
    fs_read["ImageSize"] >> imagesize;
    tRes.iIndex      = 0xff;
    tRes.iWidth      = imagesize.at(0);
    tRes.iHeight     = imagesize.at(1);
    tRes.iWidthFOV   = imagesize.at(0);
    tRes.iHeightFOV  = imagesize.at(1);
    tRes.iHOffsetFOV = 0;
    tRes.iVOffsetFOV = 0;

    fs_read["Gamma"] >> camera_gamma;

    fs_read.release();
}
bool MindvisionCamera::CameraParamSet()
{

    bool state = false;

    state = SetFrameSpeed();
    if (state == false)
    {
        std::cout << "FrameSpeed set error !" << std::endl;
    }

    state = SetExposureTime(exposure_time);
    if (state == false)
    {
        std::cout << "SetExposureTime error !" << std::endl;
    }

    state = SetGamma(camera_gamma);
    if (state == false)
    {
        std::cout << "SetGamma error !" << std::endl;
    }

    state = SetGain(iRGain, iGGain, iBGain);
    if (state == false)
    {
        std::cout << "SetGain error !" << std::endl;
    }

    state = SetSize();
    if (state == false)
    {
        std::cout << "SetSize error !" << std::endl;
    }
}

bool MindvisionCamera::runCamera()
{
    CameraParamInit(1);
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);

    //没有连接设备
    if (iCameraCounts == 0)
    {
        return -1;
    }

    //相机初始化
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    //初始化失败
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }

    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = ( unsigned char* )malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    CameraPlay(hCamera);

    //设置参数
    iStatus = CameraParamSet();

    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }
}
bool MindvisionCamera::Do()
{
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
    {
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

        cv::Mat matImage(cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight), sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3, g_pRgbBuffer);

        cvtColor(matImage, src_, CV_RGB2BGR);
        //        imshow("src_", src_);
        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
}
bool MindvisionCamera::getImage(cv::Mat& src)
{

    src = src_.clone();
    return true;
}

bool MindvisionCamera::SetExposureTime(double exposure_time)
{
    bAeState = false;
    iStatus  = CameraSetAeState(hCamera, FALSE);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "设置曝光模式失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "设置曝光模式成功" << std::endl;
        if (bAeState == true)
        {
            std::cout << "当前为自动曝光模式" << std::endl;
        }
        else
        {
            std::cout << "自动曝光模式关闭" << std::endl;
        }
    }
    iStatus = CameraSetExposureTime(hCamera, exposure_time);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "设置曝光失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "设置曝光成功，当前曝光为" << exposure_time << std::endl;
        return true;
    }
}
bool MindvisionCamera::SetGain(int iRGain, int iGGain, int iBGain)
{

    iStatus = CameraSetGain(hCamera, iRGain, iGGain, iBGain);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "设置增益失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "设置增益成功，当前增益为 R " << iRGain << " G " << iGGain << " B " << iBGain << std::endl;
        return true;
    }
}
bool MindvisionCamera::SetGamma(int camera_gamma)
{

    iStatus = CameraSetGamma(hCamera, camera_gamma);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "设置伽马失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "设置伽马成功，当前伽马为" << camera_gamma << std::endl;
        return true;
    }
}
bool MindvisionCamera::SetSize()
{

    iStatus = CameraSetImageResolution(hCamera, &tRes);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "Set image_size error !" << std::endl;
        return false;
    }
    else
    {
        std::cout << "Set image_size true,width= " << tRes.iWidth << " height: " << tRes.iHeight << std::endl;
    }
}
bool MindvisionCamera::SetFrameSpeed()
{
    iStatus = CameraSetFrameSpeed(hCamera, iFrameSpeed);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        std::cout << "设置帧率失败" << std::endl;
        return false;
    }
    else
    {
        std::cout << "设置帧率成功，当前帧率为" << iFrameSpeed << std::endl;
        return true;
    }
}
