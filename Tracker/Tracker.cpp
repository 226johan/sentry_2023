#include "Tracker.h"

void Tracker::setTrackerROI(cv::Mat& src,cv::Mat& tracker_roi){

    //设置跟踪区域
     if (last_armor_rrect_.center.x == 0 || last_armor_rrect_.center.y == 0){
         tracker_roi = src;
         track_ROI_ = cv::Rect(0, 0, src.cols, src.rows);
     }
     else{
         if (if_track){
             cv::Rect rect = last_armor_rrect_.boundingRect();  //将旋转矩用直立矩形框起来是为了跟踪目标定位
             int      w    = int(rect.width * width_scale_);
             int      h    = int(rect.height * height_scale_);

             cv::Point center     = last_armor_rrect_.center;
             int       x          = std::max(center.x - w, 0);
             int       y          = std::max(center.y - h, 0);  //将跟踪便捷放大，同时相同中心放大
             cv::Point up_left    = cv::Point(x, y);
             x                    = std::min(center.x + w, src.cols);
             y                    = std::min(center.y + h, src.rows);
             cv::Point down_right = cv::Point(x, y);

             track_ROI_ = cv::Rect(up_left, down_right);  //设置跟踪区域

             ///< 判断装甲是不是超出范围
             if (makeRectSafe(track_ROI_, src.size()) == false){
                 last_armor_rrect_ = cv::RotatedRect();
                 track_ROI_        = cv::Rect(0, 0, src.cols, src.rows);
             }
             else{
                 tracker_roi = src(track_ROI_);
                 rectangle(src, track_ROI_, cv::Scalar(255, 255, 0), 2, 8);  //画出跟踪区域
             }
         }
         else{
             tracker_roi = src;
             tracker_count++;
         }
     }
}

void  Tracker::judgeIfTrack(cv::Mat& src){

    tracker_count++;
    armor_drop_frame_count = 5;
    if (tracker_count < 5){
        if_track = false;
    }
    else{
        if_track = true;
        putText(src, "Tracking Mode", cv::Point2f(500, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

}

void  Tracker::stopTrack(cv::Mat& src){

//    tracker_count   = 0;  //丢失对象
//    if_track        = false;
//    if_first_frame  = true;

    armor_lost_frame_count++; ///< 如果连续5帧没有识别到装甲板，判断为装甲板丢失
    if(armor_lost_frame_count >= 5 ){
        if_armors_found = false;
        if_track = false;
        putText(src, "No Armor", cv::Point2f(400, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        if_first_frame  = true;
//      armor_detetor.tracker.last_armor_rrect_ = cv::RotatedRect();
    }
    else{
        if_armors_found = true;
        if_track = true;
    }
}


/**
 * @brief 确保矩形框正常
 * @param <rect> 所选的矩形框
 * @param <size> 矩形框大小
 * @return true-正常 false-不正常
 */
bool Tracker::makeRectSafe(cv::Rect& rect, cv::Size size){
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}
