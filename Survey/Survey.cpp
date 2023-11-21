#include <Survey/Survey.h>

Survey::Survey()
{
    ///< 构造函数初始化相机内参 960 900
    /// 1950 1950
    cameraMatrix = (cv::Mat_<double>(3, 3) << 2650, 0, 600, 0, 2650, 512, 0, 0, 1);  //  哨兵 1950 320 256 新步兵 1415.0 480 450    1755，0，320，0，1755，256，0，0，1
    distCoeffs   = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
}

void Survey::DistanceSolver(Armor& armor)
{
    setWorldPoints(armor.type);
    solvePNP(armor);
    calculateDistanceToCenter(armor);
    // Tramsformer_Point(armor,yaw,pitch);
}

void Survey::setWorldPoints(ArmorType armor_type)
{
    if (armor_type == SmallArmor)
    {
        target_width  = 133;
        target_height = 55;
    }
    if (armor_type == BigArmor)
    {
        target_width  = 228;
        target_height = 55;
    }

    points_in_world.clear();
    points_in_world.push_back(cv::Point3d(-target_width / 2.0, target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(-target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, -target_height / 2.0, 0));
    points_in_world.push_back(cv::Point3d(target_width / 2.0, target_height / 2.0, 0));
}

void Survey::solvePNP(Armor& armor)
{

    double x, y, z;
    solvePnP(points_in_world, armor.armor_points, cameraMatrix, distCoeffs, rotate_mat, trans_mat, false, cv::SOLVEPNP_AP3P);
    x = trans_mat.ptr<double>(0)[0];
    y = trans_mat.ptr<double>(0)[1];
    z = trans_mat.ptr<double>(0)[2];

    armor.pose.translation().x() = trans_mat.ptr<double>(0)[0];
    armor.pose.translation().y() = trans_mat.ptr<double>(0)[1];
    armor.pose.translation().z() = trans_mat.ptr<double>(0)[2];

    // rvec to 3x3 rotation matrix
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotate_mat, rotation_matrix);

    Eigen::Matrix3d tf2_rotation_matrix;
    tf2_rotation_matrix << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1),
        rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);
    armor.pose.linear() = tf2_rotation_matrix;

    armor.distance = sqrtf(x * x + y * y + z * z);
}

void Survey::calculateDistanceToCenter(Armor& armor)
{
    float cx                       = cameraMatrix.at<double>(0, 2);
    float cy                       = cameraMatrix.at<double>(1, 2);
    distance_to_image_center       = cv::norm(armor.center - cv::Point2f(cx, cy));
    armor.distance_to_image_center = distance_to_image_center;
}

void Survey::Tramsformer_Point(Armor& armor, double yaw, double pitch)
{
    Eigen::Affine3d p;
    p = armor.pose;
    //将欧拉角转换为旋转矩阵
    Eigen::Matrix4d rotation_matrix;

    rotation_matrix = computeTransformationMatrix(yaw, pitch);

    Eigen::Matrix4d temp;
    temp.block<3, 3>(0, 0) = p.linear();
    temp.block<3, 1>(0, 3) = p.translation();

    Eigen::Matrix4d dst;
    dst = rotation_matrix.inverse() * temp;

    armor.pose_tramsformer.linear()      = dst.block<3, 3>(0, 0);
    armor.pose_tramsformer.translation() = dst.block<3, 1>(0, 3);
}

void Survey::AimTraget(std::vector<Armor>& armors)
{
    Armor temp = armors[0];
    for (auto& armor : armors)
    {
        if (armor.distance_to_image_center < temp.distance_to_image_center)
        {
            temp = armor;
        }
    }
    std::cout << "11111111" << std::endl;
    x_            = temp.pose.translation().x();
    y_            = temp.pose.translation().y();
    z_            = temp.pose.translation().z();
    send_distance = sqrtf(x_ * x_ + y_ * y_ + z_ * z_);
    std::cout << "send_distance:" << send_distance;

    ///< 使用重力模型，获取发送的yaw和pitch
    init();
    send_yaw = (-atan(x_ / send_distance) + 0.028f) * 180 / CV_PI;
    std::cout << "send_yaw:" << send_yaw;

    //    if(send_distance < 3000)
    //    {
    send_pitch = Transform(z_, y_ - 50.0f) * 180 / CV_PI;
    //    }
    //    else if (send_distance > 4000 and send_distance < 6000)
    //    {
    //        send_pitch = Transform(z_, y_- 280.0f)* 380 / CV_PI;
    //    }else if(send_distance > 6000)
    //    {
    //        send_pitch = Transform(z_, y_- 300.0f)* 1000 / CV_PI;
    //    }
    //    else
    //    {
    //        send_pitch = Transform(z_, y_- 50.0f)* 260 / CV_PI;
    //    }
    std::cout << "send_pitch:" << send_pitch;
}

Eigen::Matrix4d Survey::computeTransformationMatrix(double& yaw, double& pitch)
{
    // Convert angles to radians
    double yawRad   = yaw * M_PI / 180.0;
    double pitchRad = pitch * M_PI / 180.0;

    // Compute rotation matrices
    Eigen::Matrix3d rotationYaw   = Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rotationPitch = Eigen::AngleAxisd(pitchRad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotationZ     = Eigen::AngleAxisd(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rotationX     = Eigen::AngleAxisd(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // Compute translation vector
    Eigen::Vector3d translation(0.03, 0.0, 0.02);

    // Create transformation matrix
    Eigen::Matrix4d transformationMatrix   = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationX * rotationZ * rotationPitch * rotationYaw;
    transformationMatrix.block<3, 1>(0, 3) = translation;

    return transformationMatrix;
}

Eigen::Matrix4d Survey::computeTransformationMatrix1(double& yaw, double& pitch)
{
    // Convert angles to radians
    double yawRad   = yaw * M_PI / 180.0;
    double pitchRad = pitch * M_PI / 180.0;

    // Compute rotation matrices
    Eigen::Matrix3d rotationYaw   = Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rotationPitch = Eigen::AngleAxisd(pitchRad, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotationZ     = Eigen::AngleAxisd(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rotationX     = Eigen::AngleAxisd(-90.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // Compute translation vector
    Eigen::Vector3d translation(0.10, 0.0, 0.08);

    // Create transformation matrix
    Eigen::Matrix4d transformationMatrix   = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationYaw * rotationPitch * rotationZ * rotationX;
    transformationMatrix.block<3, 1>(0, 3) = translation;

    return transformationMatrix;
}
