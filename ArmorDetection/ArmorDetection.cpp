#include "fstream"
#include <ArmorDetection/ArmorDetection.h>
Light Armor::extendLight(const Light& old_light)
{
    Light new_light;
    new_light.top        = old_light.top;
    new_light.bottom     = old_light.bottom;
    new_light.length     = old_light.length;
    new_light.width      = old_light.width;
    new_light.tilt_angle = old_light.tilt_angle;

    new_light.angle  = old_light.angle;
    new_light.center = old_light.center;

    if (old_light.size.width < old_light.size.height)
    {
        new_light.size.height = 2 * old_light.size.height;
        new_light.size.width  = old_light.size.width;
    }
    else
    {
        new_light.size.width  = 2 * old_light.size.width;
        new_light.size.height = old_light.size.height;
    }
    return new_light;
}

void Armor::sortLightPoints(cv::Point2f points[4], std::vector<cv::Point2f>& new_points)
{
    cv::Point2f temp_point;
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = i + 1; j < 4; j++)
        {
            if (points[i].y > points[j].y)
            {
                temp_point = points[i];
                points[i]  = points[j];
                points[j]  = temp_point;
            }
        }
    }
    if (points[0].x > points[1].x)
    {
        temp_point = points[0];
        points[0]  = points[1];
        points[1]  = temp_point;
    }
    if (points[2].x < points[3].x)
    {
        temp_point = points[2];
        points[2]  = points[3];
        points[3]  = temp_point;
    }
    for (size_t i = 0; i < 4; i++)
    {
        new_points.push_back(points[i]);
    }
}

ArmorDetetion::ArmorDetetion()
{
    net_ = cv::dnn::readNetFromONNX("/home/qingling/thread/sentry7/Params/fc.onnx");
    std::ifstream label_file("/home/qingling/thread/sentry7/Params/label.txt");
    std::string   line;
    while (std::getline(label_file, line))
    {
        class_names_.push_back(line);
    }
}

bool ArmorDetetion::ArmorDeteted(cv::Mat& src)
{
    ImageProsser(src);  //->Dst_Mix
    if (FindLight(src) == false)
    {
        std::cout << "没有找到灯条" << std::endl;
        return false;
    }
    if (FindArmor(src) == false)
    {
        std::cout << "没有找到装甲板" << std::endl;
        return false;
    }

    Digital_recognition(src, found_armors, target_armors);

    //确定目标装甲板
    if (DrawTarget(src, target_armors) == false)
    {
        std::cout << "数字识别没识别到数字" << std::endl;
        return false;
    }
    return true;
}

//图像预处理
void ArmorDetetion::ImageProsser(cv::Mat& src)
{
    target_armors.clear();
    param.AdjustParam();  //图像预处理调节滑块条

    cv::Mat src_show = src;
    Dst_Mix          = cv::Mat::zeros(src_show.size(), CV_8UC1);

    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    /*****************************************红色*********************************************/
    if (enemy_color == Red)
    {
        cv::Mat              src_gray_threshold;
        std::vector<cv::Mat> splited_channels;
        split(src_show, splited_channels);                           ///< 分离通道
        cvtColor(src_show, src_gray_threshold, cv::COLOR_BGR2GRAY);  ///< 灰度化
        threshold(src_gray_threshold, src_gray_threshold, 96, 255, cv::THRESH_BINARY);
        // imshow("灰度图阈值化", src_gray_threshold);

        cv::Mat src_subtract_threshold;
        subtract(splited_channels[2], splited_channels[0], src_subtract_threshold);  ///< 通道相减
        threshold(src_subtract_threshold, src_subtract_threshold, 112, 255, cv::THRESH_BINARY);
        // imshow("通道相减阈值化", src_subtract_threshold);

        dilate(src_subtract_threshold, src_subtract_threshold, element);  ///< 膨胀
        Dst_Mix = src_subtract_threshold & src_gray_threshold;            ///< 灰度图阈值化和通道相减阈值化相与
        //延长灯条
        FillHole(Dst_Mix, Dst_Mix);  ///< 漫水填充
                                     // namedWindow("图像预处理最终-红色", cv::WINDOW_AUTOSIZE);
        imshow("图像预处理最终-红色", Dst_Mix);
    }
    /*****************************************蓝色*********************************************/
    else if (enemy_color == Blue)
    {

        cv::Mat              src_gray_threshold;
        std::vector<cv::Mat> splited_channels;
        split(src_show, splited_channels);                           ///< 分离通道
        cvtColor(src_show, src_gray_threshold, cv::COLOR_BGR2GRAY);  ///< 灰度化
        //170,255
        threshold(src_gray_threshold, src_gray_threshold, 170, 255, cv::THRESH_BINARY);
        // imshow("灰度图阈值化", src_gray_threshold);

        cv::Mat src_subtract_threshold;
        subtract(splited_channels[0], splited_channels[2], src_subtract_threshold);  ///< 通道相减
        //100,255
        threshold(src_subtract_threshold, src_subtract_threshold, 110, 255, cv::THRESH_BINARY);
        //  imshow("通道相减阈值化", src_subtract_threshold);

        dilate(src_subtract_threshold, src_subtract_threshold, element);  ///< 膨胀
        Dst_Mix = src_subtract_threshold & src_gray_threshold;            ///< 灰度图阈值化和通道相减阈值化相与
        //延长灯条
        FillHole(Dst_Mix, Dst_Mix);  ///< 漫水填充
                                     //  namedWindow("图像预处理最终-蓝色", cv::WINDOW_AUTOSIZE);
        imshow("图像预处理最终-蓝色", Dst_Mix);
    }
}

void ArmorDetetion::FillHole(cv::Mat srcBw, cv::Mat& dstBw)
{
    cv::Mat broedMat = cv::Mat::zeros(srcBw.rows + 2, srcBw.cols + 2, CV_8UC1);
    srcBw.copyTo(broedMat(cv::Range(1, srcBw.rows + 1), cv::Range(1, srcBw.cols + 1)));
    cv::floodFill(broedMat, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat cutMat;
    broedMat(cv::Range(1, srcBw.rows + 1), cv::Range(1, srcBw.cols + 1)).copyTo(cutMat);
    dstBw = srcBw | (~cutMat);
}

bool ArmorDetetion::FindLight(cv::Mat& src)
{
    cv::Mat src_show = src.clone();

    found_lights.clear();  ///< 清空灯条容器

    std::vector<std::vector<cv::Point2i>> contours;
    std::vector<cv::Vec4i>                hierarchy;
    findContours(Dst_Mix, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours)
    {
        cv::RotatedRect rrect = minAreaRect(contour);
        auto            light = Light(rrect);
        //灯条筛选
        if (isLight(light))
        {
            found_lights.push_back(light);
        }
    }

    //________________________________________________________________________________________________
    if (found_lights.size() == 0)
    {
        std::cout << "未找到灯条" << std::endl;
        // imshow("灯条显示", src_show);
        return false;
    }
    else
    {
        for (auto found_light : found_lights)
        {
            cv::Point2f light_points[4];
            found_light.points(light_points);
            for (uint8_t i = 0; i < 4; i++)
            {
                line(src_show, light_points[i], light_points[(i + 1) % 4], cv::Scalar(0, 255, 0), 1, 8);
            }
            imshow("灯条显示", src_show);
        }
        return true;
    }
}

bool ArmorDetetion::isLight(const Light& light)
{
    // The ratio of light (short side / long side)
    float ratio = light.width / light.length;
    // std::cout<<"ratio="<<ratio<<std::endl;
    bool ratio_ok = 0.05f < ratio && ratio < 0.65f;

    bool angle_ok = light.tilt_angle < 40.0f;
    // std::cout<<"light.tilt_angle="<<light.tilt_angle<<std::endl;
    bool is_light = ratio_ok && angle_ok;
    //    std::cout<<"______________________________________"<<std::endl;

    return is_light;
}

bool ArmorDetetion::FindArmor(cv::Mat& src)
{
    cv::Mat src_imshow = src.clone();
    found_armors.clear();  // 清空装甲板容器

    for (auto light_1 = found_lights.begin(); light_1 != found_lights.end(); light_1++)
    {
        for (auto light_2 = light_1 + 1; light_2 != found_lights.end(); light_2++)
        {
            auto armor = Armor(*light_1, *light_2);
            if (isArmor(armor))
            {
                found_armors.emplace_back(armor);
            }
        }
    }

    if (found_armors.size() == 0)
    {
        std::cout << "灯条不匹配，未找到装甲板" << std::endl;
        // imshow("装甲板显示", src_imshow);
        return false;
    }
    else
    {
        for (size_t i = 0; i < found_armors.size(); i++)
        {
            for (uint8_t j = 0; j < 4; j++)
            {
                line(src_imshow, found_armors[i].armor_points[j], found_armors[i].armor_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, 8);
            }
            imshow("装甲板显示", src_imshow);
        }
        return true;
    }
}

//装甲板筛选
bool ArmorDetetion::isArmor(Armor& armor)
{
    Light light_1 = armor.left_light;
    Light light_2 = armor.right_light;

    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
    // std::cout<<"light_length_ratio ="<<light_length_ratio<<std::endl;

    //0.8
    bool light_ratio_ok = light_length_ratio > 0.7 /*a.min_light_ratio*/;
    std::cout<<"light_ratio "<<light_length_ratio<<std::endl;
    float avg_light_length = (light_1.length + light_2.length) / 2;

//     std::cout<<"avg_light_length ="<<avg_light_length<<std::endl;

    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;

//     std::cout<<"center_distance = "<<center_distance<<std::endl;

     //0.5 4.0
    bool center_distance_ok = (/*a.min_small_center_distance*/ 0.5 < center_distance && center_distance < 4.5 /*a.max_small_center_distance*/); /*||*/
    //                            (/*a.min_large_center_distance*/3.2 < center_distance /*&&
    //                             center_distance <4.0 *//*a.max_large_center_distance*/);

    cv::Point2f diff     = light_1.center - light_2.center;
    float       angle    = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;

    //8.0
    bool        angle_ok = angle < /*a.max_angle*/ 8.0;
//     std::cout<<"angle="<<angle<<std::endl;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
    // std::cout<<"is_amor =="<<is_armor <<std::endl;

    //根据装甲板长宽比判断装甲板类型
    armor.type = center_distance > /*a.min_large_center_distance*/ 3.2 ? BigArmor : SmallArmor;
//    std::cout<<"center_distence: "<<center_distance<<std::endl;
    while(0)
    {
        cv::Mat armor_frame(640,512,CV_8UC3,cv::Scalar(0,0,0));
        putText(armor_frame,"light_length_ratio :"+std::to_string(light_length_ratio),cv::Point(4,15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        putText(armor_frame,"center_distance :"+std::to_string(center_distance),cv::Point(4,35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        putText(armor_frame,"angle :"+std::to_string(angle),cv::Point(4,55), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    }
    return is_armor;

}

void ArmorDetetion::Digital_recognition(const cv::Mat& src, std::vector<Armor> armors, std::vector<Armor>& target_armors)
{
    extractNumbers(src, found_armors);
    classify(found_armors, target_armors);
}

void ArmorDetetion::extractNumbers(const cv::Mat& src, std::vector<Armor>& armors)
{
    cv::Mat src_show = src.clone();
    for (auto& armor : armors)
    {
        ///< 去除灯条之后的装甲数字、仅仅保存数字部分,仿射变换
        cv::Point2f affinePoints[4] =
            {

                //1     2
                //0     3
                // 1230

                cv::Point2f(armor.new_armor_points[0].x + 15, armor.new_armor_points[0].y),  // 15    1
                cv::Point2f(armor.new_armor_points[1].x - 10, armor.new_armor_points[1].y),  // 10    2
                cv::Point2f(armor.new_armor_points[2].x - 10, armor.new_armor_points[2].y),  // 10    3
                cv::Point2f(armor.new_armor_points[3].x + 15, armor.new_armor_points[3].y),  // 15    0
            };                                                                               // 7

        cv::putText(src_show, "22", cv::Point2f(armor.armor_points[2].x, armor.armor_points[2].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 155, 238), 1);
        cv::putText(src_show, "00", cv::Point2f(armor.armor_points[0].x, armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 155, 238), 1);
        cv::putText(src_show, "11", cv::Point2f(armor.armor_points[1].x, armor.armor_points[1].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 155, 238), 1);
        cv::putText(src_show, "33", cv::Point2f(armor.armor_points[3].x, armor.armor_points[3].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 155, 238), 1);

        cv::putText(src_show, "0", cv::Point2f(armor.new_armor_points[0].x, armor.new_armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(165, 155, 238), 1);
        cv::putText(src_show, "1", cv::Point2f(armor.new_armor_points[1].x, armor.new_armor_points[1].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(165, 155, 238), 1);
        cv::putText(src_show, "2", cv::Point2f(armor.new_armor_points[2].x, armor.new_armor_points[2].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(165, 155, 238), 1);
        cv::putText(src_show, "3", cv::Point2f(armor.new_armor_points[3].x, armor.new_armor_points[3].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(165, 155, 238), 1);
        cv::Point2f affinePoints2[4] = { cv::Point2f(0, 0), cv::Point2f(48, 0), cv::Point2f(48, 48), cv::Point2f(0, 48) };
        cv::imshow("src_show",src_show);
        cv::Mat affine_trans;
        //透视变换
        affine_trans = getPerspectiveTransform(affinePoints, affinePoints2);
        cv::Mat number_image;
        //仿射变换
        cv::warpPerspective(src, number_image, affine_trans, cv::Size(48, 48), cv::INTER_CUBIC);
        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
//        cv::threshold(number_image, number_image, 80, 240, cv::THRESH_BINARY);
        armor.number_img = number_image;
    }
}

//分类
void ArmorDetetion::classify(std::vector<Armor>& armors, std::vector<Armor>& target_armors)
{
    for (auto& armor : armors)
    {
        target_armors.clear();
        cv::Mat image = armor.number_img.clone();

        image = image / 255.0;

        cv::Mat blob;
        //20,28
        cv::dnn::blobFromImage(image, blob, 1.0, cv::Size(20, 28));

        net_.setInput(blob);

        cv::Mat outputs = net_.forward();

        float   max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double    confidence;
        cv::Point class_id_point;
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number     = class_names_[label_id];

        std::stringstream result_ss;
        result_ss << armor.number << ": " << std::fixed << std::setprecision(1) << armor.confidence * 100.0 << "%";
        armor.classfication_result = result_ss.str();
        cv::imshow("11",armor.number_img);
//        cout<<"confidence: "<confidence<< endl;
        if (confidence * 100.0 > 10 && class_names_[label_id] != "negative")
        {
            target_armors.push_back(armor);
            cv::imshow("1", armor.number_img);
        }
    }
}

bool ArmorDetetion::DrawTarget(cv::Mat& src, std::vector<Armor>& target_armors)
{

    Armor target_armor;
    if (target_armors.size() == 0)
    {
        std::cout << "0.没有目标装甲板" << std::endl;
        return false;
    }
    else if (target_armors.size() == 1)
    {

//        if (target_armors[0].type == SmallArmor && target_armors[0].number == "2")
//        {
//            putText(src, target_armor.classfication_result, cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
//            std::cout << "2.没有目标装甲板" << std::endl;
//            return true;
//        }
//        else
//        {
            target_armor = target_armors[0];

            putText(src, target_armor.classfication_result, cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
            if (target_armor.type == SmallArmor)
            {
                putText(src, "small_armor", cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                for (size_t k = 0; k < 4; k++)
                {
                    line(src, target_armor.armor_points[k], target_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                }
            }
            else if (target_armor.type == BigArmor)
            {
                putText(src, "big_armor", cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                for (size_t k = 0; k < 4; k++)
                {
                    line(src, target_armor.armor_points[k], target_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                }
            }
//        }
        return true;
    }
    else
    {  //多目标
        for (size_t i = 0; i < target_armors.size(); i++)
        {
            if (target_armors[i].type == BigArmor && target_armors[i].number == "1")
            {
                //优先攻击英雄
                target_armor = target_armors[i];

                putText(src, target_armor.classfication_result, cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(165, 155, 238),
                        1);
                for (size_t k = 0; k < 4; k++)
                {
                    line(src, target_armor.armor_points[k], target_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                }
                if (target_armor.type == SmallArmor)
                {
                    putText(src, "small_armor", cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                }
                else if (target_armor.type == BigArmor)
                {
                    putText(src, "big_armor", cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                }
                break;
            }
            else if (target_armors[i].type == SmallArmor)
            {
                target_armor = target_armors[i];

                putText(src, target_armor.classfication_result, cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238),
                        1);
                for (size_t k = 0; k < 4; k++)
                {
                    line(src, target_armor.armor_points[k], target_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
                }
                if (target_armor.type == SmallArmor)
                {
                    putText(src, "small_armor", cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                }
                else if (target_armor.type == BigArmor)
                {
                    putText(src, "big_armor", cv::Point2f(target_armor.armor_points[0].x, target_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
                }
                break;
            }
        }
    }
    return true;
}
