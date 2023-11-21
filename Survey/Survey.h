#ifndef SURVEY_H
#define SURVEY_H

#include "AngleSolver/GimbalControl.h"
#include <ArmorDetection/ArmorDetection.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

class Survey : public GimbalControl  //测量器应该是起到承上启下的作用，上承接识别器装甲板信息（函数的输入），下接追踪器的装甲板信息（函数的输出）
{
public:
    Survey();
    void            DistanceSolver(Armor& armor);
    void            setWorldPoints(ArmorType armor_type);     //同上同上
    void            solvePNP(Armor& armor);                   //同上同上
    void            calculateDistanceToCenter(Armor& armor);  //同上同上
    void            AimTraget(std::vector<Armor>& armors);
    void            Tramsformer_Point(Armor& armor, double yaw, double pitch);
    Eigen::Matrix4d computeTransformationMatrix(double& yaw, double& pitch);
    Eigen::Matrix4d computeTransformationMatrix1(double& yaw, double& pitch);

    std::vector<cv::Point3d> points_in_world;
    //_______________________________________________________________________________________________
    double target_width;
    double target_height;
    //_______________________________________________________________________________________________
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat rotate_mat;
    cv::Mat trans_mat;
    //_______________________________________________________________________________________________
    float send_yaw;
    float send_pitch;
    float bullet_flight_time;

    //______________________________________________________________________________________________
    double distance_to_image_center;
    //______________________________________________________________________________________________

    cv::Mat                  cameraMatrix_;     ///<相机内参矩阵
    cv::Mat                  distCoeffs_;       ///<畸变系数
    double                   target_width_;     ///<世界坐标下装甲板的宽，毫米
    double                   target_height_;    ///<世界坐标下装甲板的高，毫米
    std::vector<cv::Point3d> points_in_world_;  ///<以装甲板中心为零点，建立x,y坐标系
    cv::Mat                  rotate_mat_;       ///<旋转矩阵
    cv::Mat                  trans_mat_;        ///<平移矩阵
    double                   x_, y_, z_;        ///<平移量（状态量）

    float gain_yaw;    ///< 获取的yaw
    float gain_pitch;  ///< 获取的pitch

    //    float send_yaw;       ///< 发送的yaw
    //    float send_pitch;     ///< 发送的pitch
    float send_distance;  ///< 发送的distance

    float original_yaw;
    float original_pitch;

    bool if_shoot;  ///< 是否射击标志位(哨兵用)
};

#endif
