#include "MultiThreading.h"
#include <fstream>
Mat src;
Mat text_frame;

///多线程
pthread_t ImagePro;
pthread_t Camera;
pthread_t Carmonitor;
pthread_t Serial;

///迈德威视
MindvisionCamera camera_mindvision;

///串口
Uart InfoPort;
int  fd_serial0   = 0;
bool serial_state = 0;

HostComputerData  RobotInfo;        //发送
GroundChassisData MainControlInfo;  //接收

//定义检测类对象
ArmorDetetion armordetetion;
//定义解算器
Survey survey;
//定义追踪器
Tracker tracker;

Kalmanfilter kalman;
float        time_ = 0;
double       fps;

//文件管理
ofstream out_pitch;
ofstream out_yaw;
ofstream out_shoot;

void* CameraMindvision(void*)
{
    camera_mindvision.runCamera();
}

void* ImageProcess(void*)
{

    //    camera_mindvision.runCamera();
    while (1)
    {
        Mat temp(640, 512, CV_8UC3, Scalar(0, 0, 0));
        text_frame                                  = temp.clone();
        std::chrono::steady_clock::time_point time1 = std::chrono::steady_clock::now();
        camera_mindvision.Do();
        camera_mindvision.getImage(src);

        //        sem_wait(&emptysend);
        if (!src.empty())
        {
            //从串口中获取敌方机器人模式
            if (static_cast<int>(MainControlInfo.color) == 0 || static_cast<int>(MainControlInfo.color) == 1)
            {
                if (static_cast<int>(MainControlInfo.color) == 0)
                {
                    armordetetion.enemy_color = Red;
                }
                if (static_cast<int>(MainControlInfo.color) == 1)
                {
                    armordetetion.enemy_color = Blue;
                }
            }

            armordetetion.enemy_color = Red;

            if (armordetetion.ArmorDeteted(src))
            {
                tracker.judgeIfTrack(src);

                /******************************************************************/
                //检测串口
                putText(src, "Detection:", cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                putText(src, "Gein_yaw:" + std::to_string(MainControlInfo.gain_yaw.f), cv::Point2f(5, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                putText(src, "Gein_pitch:" + std::to_string(MainControlInfo.gain_pitch.f), cv::Point2f(5, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                /******************************************************************/

                for (auto& target_armor : armordetetion.target_armors)
                {
                    survey.DistanceSolver(target_armor);
                }
                survey.AimTraget(armordetetion.target_armors);

                RobotInfo.Yaw.f    = survey.send_yaw * 1.0f;
                RobotInfo.Pitch.f  = survey.send_pitch * 1.0f;
                RobotInfo.if_shoot = 1;

                tracker.last_send_yaw   = RobotInfo.Yaw.f;
                tracker.last_send_pitch = RobotInfo.Pitch.f;
            }
            else
            {
                tracker.armor_drop_frame_count--;
                if (tracker.armor_drop_frame_count <= 5 && tracker.armor_drop_frame_count > 0)
                {  //处于掉帧模式
                    putText(src, "Lost Frame Mode", cv::Point2f(500, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    RobotInfo.Yaw.f   = tracker.last_send_yaw;
                    RobotInfo.Pitch.f = tracker.last_send_pitch;
                    //                RobotInfo.if_armor_exist = 1;
                }
                if (tracker.armor_drop_frame_count < 0)
                {  //连续5帧未识别到，退出掉帧模式
                    tracker.if_first_frame = true;
                    RobotInfo.Yaw.f        = 0.f;
                    RobotInfo.Pitch.f      = 0.f;
                    RobotInfo.if_shoot     = 0;
                }
            }

            RobotInfo.Yaw.f   = RobotInfo.Yaw.f * 0.8f;
            RobotInfo.Pitch.f = RobotInfo.Pitch.f;

            std::cout << "gain_yaw " << RobotInfo.Yaw.f << std::endl;
            std::cout << "gain_pitch " << RobotInfo.Pitch.f << std::endl;

            // kamral
            if (1)
            {
                if (RobotInfo.Yaw.f != 0.0f && RobotInfo.Pitch.f != 0.0f)
                {

                    survey.bullet_flight_time = ((survey.send_distance * 0.001f) / 30.0f /*static_cast<float>(MainControlInfo.speed - 1)*/);

                    kalman.gain_yaw   = MainControlInfo.gain_yaw.f * 1.55;    // 1.3  1.75
                    kalman.gain_pitch = MainControlInfo.gain_pitch.f * 1.25;  // 1.65

                    kalman.send_yaw   = RobotInfo.Yaw.f;
                    kalman.send_pitch = RobotInfo.Pitch.f;

                    if (tracker.if_first_frame == true)
                    {
                        tracker.if_first_frame = false;
                        kalman.initKalman();
                    }
                    else
                    {
                        std::cout << "time " << time_ << std::endl;
                        kalman.getUpdate(time_);
                    }
                    kalman.KalmanPredict(survey.bullet_flight_time * 1.2f);
                    putText(src, "v_yaw   = " + std::to_string(kalman.v_yaw), cv::Point2f(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    putText(src, "a_yaw   = " + std::to_string(kalman.a_yaw), cv::Point2f(10, 320), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                    RobotInfo.Yaw.f   = kalman.send_yaw;
                    RobotInfo.Pitch.f = kalman.send_pitch;

                    //                    std::cout << "卡尔曼后send_yaw:" << kalman.send_yaw << std::endl;
                    //                    std::cout << "卡尔曼后yaw:" << kalman.send_yaw + kalman.gain_yaw << std::endl;
                    //                    std::cout << "卡尔曼后pitch:" << kalman.send_yaw + kalman.gain_pitch << std::endl;
                }
                //< 排除数据异常情况,防止机器人出现意外情况
                if ((fabs(RobotInfo.Yaw.f) < 0.01) || (fabs(RobotInfo.Yaw.f) > 25.0) || (isnan(RobotInfo.Yaw.f)))
                {
                    RobotInfo.Yaw.f = 0;
                }
                if ((fabs(RobotInfo.Pitch.f) < 0.1) || (fabs(RobotInfo.Pitch.f) > 25.0) || (isnan(RobotInfo.Pitch.f)))
                {
                    RobotInfo.Pitch.f = 0;
                }

                //--程序结束时间
                std::chrono::steady_clock::time_point time2     = std::chrono::steady_clock::now();
                std::chrono::duration<double>         time_used = std::chrono::duration_cast<std::chrono::duration<double>>(time2 - time1);
                time_                                           = static_cast<float>(time_used.count());

                putText(src, "real_shoot: " + std::to_string(RobotInfo.if_real_shoot), cv::Point2f(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                fps = 1.0 / time_;
                putText(src, "Color = " + std::to_string(MainControlInfo.color), cv::Point2f(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                putText(src, "Gain_Yaw   = " + std::to_string(MainControlInfo.gain_yaw.f), cv::Point2f(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                putText(src, "Gain_Pitch = " + std::to_string(MainControlInfo.gain_pitch.f), cv::Point2f(10, 105), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                putText(src, "Send_Yaw   = " + std::to_string(RobotInfo.Yaw.f), cv::Point2f(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                putText(src, "Send_Pitch = " + std::to_string(RobotInfo.Pitch.f), cv::Point2f(10, 145), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                putText(src, "Send_Yaw_   = " + std::to_string(survey.send_yaw), cv::Point2f(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                putText(src, "Send_Pitch_ = " + std::to_string(survey.send_pitch), cv::Point2f(10, 195), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                putText(src, "Distance   = " + std::to_string(survey.send_distance / 1000.0) + " m", cv::Point2f(10, 255), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
                putText(src, "FPS = " + std::to_string(int(fps)), cv::Point2f(1150, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
                putText(src, "Time   = " + std::to_string(time_ * 1000) + " ms", cv::Point2f(10, 280), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
                line(src, Point(640, 0), Point(640, 1024), Scalar(0, 255, 255), 2, LINE_AA);
                line(src, Point(0, 650), Point(1280, 650), Scalar(0, 255, 255), 2, LINE_AA);
            }
            imshow("srcing", src);
            RobotInfo.distance.f = survey.send_distance / 1000.0;

            putText(text_frame, "mode :" + std::to_string(MainControlInfo.mode), Point(4, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            putText(text_frame, "Yaw :" + std::to_string(RobotInfo.Yaw.f), Point(4, 45), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            putText(text_frame, "Pitch :" + std::to_string(RobotInfo.Pitch.f), Point(4, 65), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            putText(text_frame, "Color :" + std::to_string(MainControlInfo.color), Point(4, 85), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            putText(text_frame, "distance :" + std::to_string(RobotInfo.distance.f), Point(4, 105), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            putText(text_frame, "gain_yaw :" + std::to_string(MainControlInfo.gain_yaw.f), Point(4, 125), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            putText(text_frame, "gain_pitch :" + std::to_string(MainControlInfo.gain_pitch.f), Point(4, 145), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            imshow("text_frame", text_frame);
        }
        else
        {
            cout << "image empty" << endl;
        }

        waitKey(10);
    }
    CameraUnInit(camera_mindvision.hCamera);
    free(camera_mindvision.g_pRgbBuffer);
}
void* GetData(void*)
{
    serial_state            = InfoPort.Init_serial(fd_serial0, 115200) + 1;
    RobotInfo.if_real_shoot = 0;
    while (1)
    {
        if (serial_state)
        {
            //接受数据
            InfoPort.GetMode(fd_serial0, MainControlInfo);

            //打印数据
            //            cout << "get_yaw: " << MainControlInfo.gain_yaw.f << endl;
            //            cout << "get_pitch: " << MainControlInfo.gain_pitch.f << endl;
            //            cout << "get_mode: " << int(MainControlInfo.mode) << endl;

            //            if(fabs(RobotInfo.Yaw.f)>5.5)
            //            {
            //                RobotInfo.Yaw.f=RobotInfo.Yaw.f*0.5;
            //            }
            //小陀螺

            //            MainControlInfo.mode=0;
            if (MainControlInfo.mode == 1)
            {

                if (RobotInfo.Yaw.f > 1.5 && RobotInfo.Yaw.f < 3)
                {
                    // 0.25
                    RobotInfo.Yaw.f = RobotInfo.Yaw.f + 1.25;
                }
            }

            // RobotInfo.Pitch.f=RobotInfo.Pitch.f*0.95;
            //前哨站
            if (RobotInfo.distance.f > 5.3 && MainControlInfo.mode == 2)
            {
                RobotInfo.Yaw.f += 0.2;
            }

            //正常巡逻
            // pitch 0.4
            if (MainControlInfo.mode != 2 && fabs(RobotInfo.Pitch.f) < 1.5)
            {

                if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 4 && RobotInfo.distance.f < 1.7)
                {
                    RobotInfo.if_real_shoot = 1;
                }
                // yaw <2.0   3.0   1.2
                else if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 3.0 && RobotInfo.distance.f < 3.5 && RobotInfo.distance.f >= 1.7)
                {
                    RobotInfo.if_real_shoot = 1;
                }
                // yaw <1.9  2.9  1.7
                else if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 3.0 && RobotInfo.distance.f >= 3.5 && RobotInfo.distance.f < 5 /*&& fabs(RobotInfo.Pitch.f)<0.4*/)
                {

                    RobotInfo.if_real_shoot = 1;
                }
                // yaw 1,3
                // pitch 0.3
                else if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 2.5 && RobotInfo.distance.f > 5 /*&& fabs(RobotInfo.Pitch.f)<0.3*/)
                {
                    //                    if(fabs(RobotInfo.Yaw.f)>5)
                    //                    {
                    //                        RobotInfo.Yaw.f=RobotInfo.Yaw.f*0.6;

                    //                    }
                    RobotInfo.if_real_shoot = 1;
                }
                else
                {
                    RobotInfo.if_real_shoot = 0;
                }
            }

            //前哨站
            else if (MainControlInfo.mode == 2)
            {
                if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 2.3 && RobotInfo.distance.f < 3.5)
                {
                    RobotInfo.if_real_shoot = 1;
                }
                else if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 1.3 && RobotInfo.distance.f >= 3.5 && RobotInfo.distance.f < 5.0)
                {

                    RobotInfo.if_real_shoot = 1;
                }

                else if (RobotInfo.if_shoot == 1 && fabs(RobotInfo.Yaw.f) < 0.8 && RobotInfo.distance.f >= 5.0 && fabs(RobotInfo.Pitch.f) < 0.2)
                {
                    RobotInfo.if_real_shoot = 1;
                }
                else
                {
                    RobotInfo.if_real_shoot = 0;
                }
            }

            if (MainControlInfo.mode == 3)
            {
                RobotInfo.if_real_shoot = 0;
            }

            //保存数据
            out_pitch.open("/home/qingling/thread/sentry7/file/out_pitch.txt", ios::app);
            out_yaw.open("/home/qingling/thread/sentry7/file/out_yaw.txt", ios::app);
            out_shoot.open("/home/qingling/thread/sentry7/file/out_shoot.txt", ios::app);
            if (out_yaw.is_open() && out_pitch.is_open() && out_shoot.is_open())
            {
                if (RobotInfo.Yaw.f != 0 && RobotInfo.Pitch.f != 0)
                {
                    out_yaw << RobotInfo.Yaw.f << " ";
                    out_pitch << RobotInfo.Pitch.f << " ";
                    out_shoot << RobotInfo.if_shoot << " ";
                }
                out_yaw.close();
                out_pitch.close();
                out_shoot.close();
            }

            //            if(RobotInfo.if_real_shoot==1)
            //            {
            //                RobotInfo.Pitch.f=RobotInfo.Pitch.f*0.8;
            //            }
            //            RobotInfo.Yaw.f=RobotInfo.Yaw.f+0.1;

            //发送数据
            InfoPort.TransformTarPos(fd_serial0, RobotInfo);
        }
    }
}

int main()
{

    pthread_create(&Camera, nullptr, CameraMindvision, nullptr);
    pthread_create(&ImagePro, nullptr, ImageProcess, nullptr);
    pthread_create(&Serial, nullptr, GetData, nullptr);

    pthread_join(Camera, nullptr);
    pthread_join(ImagePro, nullptr);
    pthread_join(Serial, nullptr);

    return 0;
}
