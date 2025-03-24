
#include "PnP_Solver.h"

PnPSolver::PnPSolver()
{
    // Unit: m
    constexpr double small_half_y = SMALL_ARMOR_WIDTH_ / 2.0 / 1000.0;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT_ / 2.0 / 1000.0;

    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up

    small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z)); //3D左下
    small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));  //3D左上
    small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));   //3D右上
    small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));  //3D右下
}

bool PnPSolver::solvePnP(const armorDetector::Armor &armor, cv::Mat &rvec, cv::Mat &tvec)
{
    std::vector<cv::Point2f> image_armor_points;

    // Fill in image points
    image_armor_points.emplace_back(armor.left_light.bottom);  // 2D 左下
    image_armor_points.emplace_back(armor.left_light.top);     // 2D 左上
    image_armor_points.emplace_back(armor.right_light.top);    // 2D 右上
    image_armor_points.emplace_back(armor.right_light.bottom); // 2D 右下
    
    
    // Solve pnp
    auto object_points = small_armor_points_;
    return cv::solvePnP(
        object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
        cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point)
{
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}