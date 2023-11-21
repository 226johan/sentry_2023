#ifndef TRACKER_H
#define TRACKER_H
#include <iostream>
#include <opencv2/opencv.hpp>

class Tracker{
public:
    void     setTrackerROI(cv::Mat& src,cv::Mat& tracker_roi);
    void     judgeIfTrack(cv::Mat& src);
    void     dropMode(cv::Mat& src);
    void     stopTrack(cv::Mat& src);
    bool     makeRectSafe(cv::Rect& rect, cv::Size size);

    double  height_scale_;  //扩大边界尺寸
    double  width_scale_;

    int     armor_lost_frame_count;   // 装甲板丢失的帧数
    int     armor_drop_frame_count;   // 装甲板掉帧模式帧数
    int     armor_found_frame_count;  // 装甲板发现的帧数


    float last_send_yaw;
    float last_send_pitch;
    float last_send_distance;

    int     tracker_count;  //跟踪计数

    cv::Rect        track_ROI_;         //跟踪区域
    cv::RotatedRect last_armor_rrect_;  //上一次的结果

    bool if_track;         //是否跟踪标志位
    bool if_armors_found;  //是否发现装甲板标志位
    bool if_first_frame;   //是否为第一帧标志位

};

#endif // TRACKER_H
