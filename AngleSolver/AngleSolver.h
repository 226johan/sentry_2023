#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#include "AngleSolver/GimbalControl.h"
#include "ArmorDetection/ArmorDetection.h"
#include <opencv2/opencv.hpp>
#include "Debug.h"

class AngleSolver : public GimbalControl
{
public:
    AngleSolver();

public:
    void operator()(std::vector<cv::Point2f> points_in_camera, ArmorType type);

    float bullet_flight_time; ///< 子弹飞行时间

    float gain_yaw;    ///< 获取的yaw
    float gain_pitch;  ///< 获取的pitch

    float send_yaw;       ///< 发送的yaw
    float send_pitch;     ///< 发送的pitch
    float send_distance;  ///< 发送的distance

    float original_yaw;
    float original_pitch;

    bool  if_shoot;       ///< 是否射击标志位(哨兵用)

private:
    void setWorldPoints(ArmorType armor_type);
    void solvePNP(std::vector<cv::Point2f> points_in_camera);

private:
    cv::Mat                  cameraMatrix_;     ///<相机内参矩阵
    cv::Mat                  distCoeffs_;       ///<畸变系数
    double                   target_width_;     ///<世界坐标下装甲板的宽，毫米
    double                   target_height_;    ///<世界坐标下装甲板的高，毫米
    std::vector<cv::Point3d> points_in_world_;  ///<以装甲板中心为零点，建立x,y坐标系
    cv::Mat                  rotate_mat_;       ///<旋转矩阵
    cv::Mat                  trans_mat_;        ///<平移矩阵
    double                   x_, y_, z_;        ///<平移量（状态量）

    float last_x_position;  ///< 记录上一次的X位置
    int   first_frame;      ///< 看是不是我们处理的第一帧数据、第一帧往往会有很多问题、不能一视同仁
    bool  if_gyro;          ///< 用于哨兵自主判断是不是小陀螺
};

#endif  // ANGLESOLVER_H
