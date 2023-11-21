#include "AngleSolver/AngleSolver.h"

AngleSolver::AngleSolver()
{
    ///< 构造函数初始化相机内参 960 900
    cameraMatrix_ = (cv::Mat_<double>(3, 3) << 1763.9458, 0, 0, 0.0003, 1763.3709, 0,415.0783,277.1733, 0.0010);  //  哨兵 1950 320 256 新步兵 1415.0 480 450     415.0783,277.1733
//    cameraMatrix_ = (cv::Mat_<double>(3, 3) << 1763.9458, 0, 0, 0.0003, 1763.3709, 0, 415.0783,277.1733, 0.0010);  //  哨兵 1950 320 256 新步兵 1415.0 480 450
    distCoeffs_   = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
}
/**
 * @brief 设置世界坐标点
 * @param 装甲类型
 * @return 世界坐标系
 */
void AngleSolver::setWorldPoints(ArmorType armor_type)
{
    if (armor_type == SmallArmor)
    {
        //        target_width_  = 133;
        //        target_height_ = 55;
        target_width_  = 133;
        target_height_ = 125;
    }
    if (armor_type == BigArmor)
    {
        //        target_width_  = 228;
        //        target_height_ = 55;
        target_width_  = 230;
        target_height_ = 128;
        points_in_world_.clear();
        points_in_world_.push_back(cv::Point3d(-target_width_ / 2.0, target_height_ / 2.0 , 0));
        points_in_world_.push_back(cv::Point3d(-target_width_ / 2.0 , -target_height_ / 2.0 , 0));
        points_in_world_.push_back(cv::Point3d(target_width_ / 2.0 , -target_height_ / 2.0 , 0));
        points_in_world_.push_back(cv::Point3d(target_width_ / 2.0 , target_height_ / 2.0 , 0));

    }
}

/**
 * @brief 获取位姿,还是计划尝试弹道模型
 * @param points_in_camera : 相机坐标点
 */
void AngleSolver::solvePNP(std::vector<cv::Point2f> points_in_camera)
{

    solvePnP(points_in_world_, points_in_camera, cameraMatrix_, distCoeffs_, rotate_mat_, trans_mat_, false, cv::SOLVEPNP_AP3P);
    x_ = trans_mat_.ptr<double>(0)[0];
    y_ = trans_mat_.ptr<double>(0)[1];
    z_ = trans_mat_.ptr<double>(0)[2];

//    if (first_frame == 1)
//    {
//        if_gyro         = false;      //第一帧不能判断是不是、因为此时数据有问题
//        last_x_position = float(x_);  //记录上次的x位姿
//    }
//    else
//    {  //此时至少是第二帧
//        //      putText(RealShow, "x_change = " + to_string(fabs(last_posx - _x)), Point2f(220, 135), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(165, 155, 238), 1);
//        ///< 通过x方向位姿变化判断陀螺模式
//        if ((fabs(last_x_position - float(x_)) > 10))
//        {
//            if_gyro = true;
//        }
//        else
//        {
//            if_gyro = false;
//        }
//        last_x_position = float(x_);  //更新此时的上次位置
//    }



    send_distance = sqrtf(x_ * x_ + y_ * y_ + z_ * z_);

    //x_=+30;
    //y_=+10;
    //z_=+0;
    ///< 使用重力模型，获取发送的yaw和pitch
    init();
    send_yaw   = -atan(x_ / z_) * 180 / CV_PI - 0.5f;

    if(send_distance < 3000){
        send_pitch = Transform(z_, y_-50.0f)* 220 / CV_PI;
    }
    else if (send_distance > 4000 and send_distance < 6000)
    {
        send_pitch = Transform(z_, y_- 280.0f)* 380 / CV_PI;
    }else if(send_distance > 6000)
    {
        send_pitch = Transform(z_, y_- 300.0f)* 1000 / CV_PI;
    }
    else{
        send_pitch = Transform(z_, y_- 50.0f)* 260 / CV_PI;
    }

//    if(send_distance < 3000){
//        send_pitch = Transform(z_, y_-50.0f)* 180 / CV_PI;
//    }
//    else if (send_distance > 4000){
//        send_pitch = Transform(z_, y_- 300.0f)* 225 / CV_PI;
//    }
//    else{
//        send_pitch = Transform(z_, y_- 180.0f)* 180 / CV_PI;
//    }
}

/**
 * @brief 角度解算最终接口
 * @param points_in_camera : 相机坐标点
 */
void AngleSolver::operator()(std::vector<cv::Point2f> points_in_camera, ArmorType armor_type)
{
    setWorldPoints(armor_type);  //依据装甲类型设置世界坐标系
    solvePNP(points_in_camera);  //计算了位姿态(x,y,z)、计算普通模式的yaw picth

}
