#include "ArmorDetection/Armorparam.h"
#include "ArmorDetection/ArmorDetection.h"

ImageProcessParam::ImageProcessParam()
{

    // read in the camera parameter yaml file 读入装甲板参数yaml文件
    cv::FileStorage fs_read("/home/qingling/thread/sentry7/Params/ArmorParams.yml", cv::FileStorage::READ);

    fs_read["red_gary_thre_value"] >> red_gary_thre_value;
    fs_read["red_minus_thre_value"] >> red_minus_thre_value;

    fs_read["blue_gary_thre_value"] >> blue_gary_thre_value;
    fs_read["blue_minus_thre_value"] >> blue_minus_thre_value;

    fs_read.release();
}

void ArmorParam::AdjustParam()
{
    namedWindow("图像预处理", cv::WINDOW_AUTOSIZE);
    //    cv::moveWindow("图像预处理", 1300, 1000);
    cv::createTrackbar("红色-灰度化->阈值化阈值", "图像预处理", &image_param.red_gary_thre_value, 255);
    cv::createTrackbar("红色-通道相减->阈值化阈值", "图像预处理", &image_param.red_minus_thre_value, 255);

    cv::createTrackbar("蓝色-灰度化->阈值化阈值", "图像预处理", &image_param.blue_gary_thre_value, 255);
    cv::createTrackbar("蓝色-通道相减->阈值化阈值", "图像预处理", &image_param.blue_minus_thre_value, 255);
}
