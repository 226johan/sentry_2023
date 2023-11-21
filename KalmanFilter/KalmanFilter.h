#ifndef KALMANTRACKER_H
#define KALMANTRACKER_H
////#include "AngleSolver/AngleSolver.h"
////#include <iostream>
////#include <opencv2/opencv.hpp>
#include "AngleSolver/AngleSolver.h"
#include "Tracker/Tracker.h"
#include <iostream>
#include <opencv2/opencv.hpp>

/**
 *@brief: kalman class
 */
class Kalmanfilter{
public:
    Kalmanfilter();
    ~Kalmanfilter();


    void initKalman();
    void getUpdate(float dT);
    void KalmanPredict(float bullet_flight_time);

//    bool processKalmanData();

//    AngleSolver angle_solver;
//    Tracker     tracker;

    cv::KalmanFilter KF;
    cv::Mat          statement;
    cv::Mat          measurement;

    float bullet_flight_time;  //子弹飞行时间  该时间*角速度
    float dT;                  // 前后帧时间差，用于计算角速度

    float v_yaw;
    float v_pitch;

    float a_yaw;
    float a_pitch;

    float gain_yaw;
    float gain_pitch;

    float send_yaw;
    float send_pitch;
};


//class Kalman_Energy
//{
//public:
//    Kalman_Energy();
//    ~Kalman_Energy();


//    void initKalman_Energy();
//    void getUpdate_Energy();
//    //    bool KalmanPredict(std::vector<cv::Point2f> now_points);
//    void KalmanPredict_Energy(float dT,float bullet_flight_time);

////    bool processKalmanData();

////    AngleSolver angle_solver;
////    Tracker     tracker;

//    cv::KalmanFilter KF;
//    cv::Mat          statement;
//    cv::Mat          measurement;

//    float bullet_flight_time;  //子弹飞行时间  该时间*角速度
//    float dT;                  // 前后帧时间差，用于计算角速度

//    float v_yaw;
//    float v_pitch;

//    float gain_yaw;
//    float gain_pitch;

//    float send_yaw;
//    float send_pitch;
//};







///**
// *@brief: kalman class
// */
//class Kalman
//{
//public:
//    Kalman();
//    ~Kalman();

//        void initKalman();
//        void getUpdate();
//        //    bool KalmanPredict(std::vector<cv::Point2f> now_points);
//        bool KalmanPredict(float dT,float bullet_flight_time);

//    //    bool processKalmanData();

//        cv::KalmanFilter KF;
//        cv::Mat          statement;
//        cv::Mat          measurement;

//        float bullet_flight_time;  //子弹飞行时间  该时间*角速度
//        float dT;                  // 前后帧时间差，用于计算角速度

//        float last_yaw = 0;
//        float last_pitch = 0;
//        float last_distance = 0;

//        float v_yaw;
//        float v_pitch;

//        float gain_yaw;
//        float gain_pitch;

//        float send_yaw;
//        float send_pitch;
//};
#endif  // KALMANTRACKER_H
