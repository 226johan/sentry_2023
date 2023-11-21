#ifndef ARMORDETECTION_H
#define ARMORDETECTION_H
#include <ArmorDetection/Armorparam.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

//灯条类（主要包含属性----灯条的上中心点、下中心点、长宽、倾斜角度）
struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width  = cv::norm(p[0] - p[1]);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    cv::Point2f p[4];
    cv::Point2f top, bottom;
    double      length, width;
    float       tilt_angle;
};

//装甲板类型（大小装甲板）
enum ArmorType
{
    SmallArmor,
    BigArmor,
};

//装甲板类（包含属性——左灯条、右灯条、装甲板中心点、两灯条上下中心四个点、
//        装甲板四个点（两灯条扩展后的）、装甲板类型、装甲板数字图像、自信度、
//        装甲板数字、分类结果、装甲板位置（在相机坐标系下）、装甲板距离图像中心的位置)
struct Armor
{
    Armor() = default;
    Armor(const Light& l1, const Light& l2)
    {
        //-----------------------------------------------------------------------
        if (l1.center.x < l2.center.x)
        {
            left_light = l1, right_light = l2;
        }
        else
        {
            left_light = l2, right_light = l1;
        }
        //-------------------------------------------------------------------------
        center = (left_light.center + right_light.center) / 2;
        armor_points.push_back(left_light.bottom);
        armor_points.push_back(left_light.top);
        armor_points.push_back(right_light.top);
        armor_points.push_back(right_light.bottom);
        //--------------------------------------------------------------------------
        new_left_light  = extendLight(left_light);  ///< 延长灯条
        new_right_light = extendLight(right_light);

        cv::Point2f left_light_points[4], right_light_points[4];
        new_left_light.points(left_light_points);
        new_right_light.points(right_light_points);

        std::vector<cv::Point2f> left_light_new_points, right_light_new_points;
        sortLightPoints(left_light_points, left_light_new_points);
        sortLightPoints(right_light_points, right_light_new_points);

        new_armor_points.push_back(left_light_new_points[1]);  // 1032  //2301
        new_armor_points.push_back(right_light_new_points[0]);
        new_armor_points.push_back(right_light_new_points[3]);
        new_armor_points.push_back(left_light_new_points[2]);
    }
    //-------------------------------------------------------------------------------
    Light                    left_light, right_light;
    cv::Point2f              center;
    std::vector<cv::Point2f> armor_points;
    //--------------------------------------------------------------------------------
    Light                    new_left_light, new_right_light;
    Light                    extendLight(const Light& old_light);
    void                     sortLightPoints(cv::Point2f points[4], std::vector<cv::Point2f>& new_points);
    std::vector<cv::Point2f> new_armor_points;

    ArmorType type;
    cv::Mat   number_img;
    //________________________________________________________________________________
    float       confidence;
    std::string number;

    std::string classfication_result;

    Eigen::Affine3d pose;
    Eigen::Affine3d pose_tramsformer;

    double distance_to_image_center;

    float distance;
};

//敌方颜色
enum EnemyColor
{
    Red,
    Blue
};

class ArmorDetetion
{
public:
    ArmorDetetion();
    //包含以下五组函数(图像处理、寻找灯条、寻找装甲板、数字识别、画出目标)
    bool ArmorDeteted(cv::Mat& src);

    //图像处理
    void ImageProsser(cv::Mat& src);
    void FillHole(cv::Mat srcBw, cv::Mat& dstBw);

    //寻找灯条
    bool FindLight(cv::Mat& src);
    bool isLight(const Light& light);

    //寻找装甲板
    bool FindArmor(cv::Mat& src);
    bool isArmor(Armor& armor);  //!!!!!!!!!!!!!!!

    //数字识别
    void Digital_recognition(const cv::Mat& src, std::vector<Armor> armors, std::vector<Armor>& target_armors);
    void extractNumbers(const cv::Mat& src, std::vector<Armor>& armors);
    void classify(std::vector<Armor>& armors, std::vector<Armor>& target_armors);

    //画出目标
    bool DrawTarget(cv::Mat& src, std::vector<Armor>& target_armors);

    //_______________________寻找灯条、寻找装甲板、数字识别、画出目标之间的函数连接所需要属性_______________________________________________________________
    cv::Mat            Dst_Mix;
    std::vector<Light> found_lights;
    std::vector<Armor> found_armors;
    std::vector<Armor> target_armors;
    //_______________________________________图像识别属性___________________________________________________
    EnemyColor enemy_color;
    ArmorParam param;
    //_______________________________________数字识别属性___________________________________________________
private:
    cv::dnn::Net             net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
};

#endif
