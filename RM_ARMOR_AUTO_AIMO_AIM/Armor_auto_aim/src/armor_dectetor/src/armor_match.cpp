// Author:xpoying
// 函数定义   预处理图像，转为灰度图后二值化，开运算，筛选出灯条，根据其匹配程度（角度差，面积差，水平程度）找对应的装甲板

#include "Armor_match.h"

armorDetector::armorDetector(double max_height_difference, double max_length_width_ratio, double min_length_width_ratio) : max_height_difference_(max_height_difference),
                                                                                                                           max_length_width_ratio_(max_length_width_ratio),
                                                                                                                           min_length_width_ratio_(min_length_width_ratio) {};
// 图像处理
void armorDetector::ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold)
{
    cv::Mat thr, gray, dil;
    cv::Mat ken = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    double threshold1 = 3;
    double threshold2 = 9;

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thr, threshold, 255, cv::THRESH_BINARY);
    cv::morphologyEx(thr, dil, cv::MORPH_OPEN, ken, cv::Point(-1, -1));
    cv::Canny(dil, edges, threshold1, threshold2);
}

// 灯条判断
bool armorDetector::IsLight(cv::RotatedRect &minRects)
{
    if (minRects.size.width > minRects.size.height)
    {
        minRects.angle += 90;
        float t = minRects.size.width;
        minRects.size.width = minRects.size.height;
        minRects.size.height = t;
    }
    if ((minRects.size.height / minRects.size.width < max_length_width_ratio_) && (min_length_width_ratio_ < minRects.size.height / minRects.size.width))
    {
        return true;
    }
    else
        return false;
}

// 装甲板标记
void armorDetector::DrawArmor(const cv::Mat &image, Armor &target_Armor)
{
    cv::RotatedRect left = target_Armor.left_light.light;
    cv::RotatedRect right = target_Armor.right_light.light;
    std::vector<cv::Point2f> leftxy(4), rightxy(4);
    right.points(rightxy.data());
    left.points(leftxy.data());
    cv::Point2f a1 = (leftxy[1] + leftxy[2]) / 2;
    cv::Point2f a2 = (leftxy[0] + leftxy[3]) / 2;
    cv::Point2f b1 = (rightxy[1] + rightxy[2]) / 2;
    cv::Point2f b2 = (rightxy[0] + rightxy[3]) / 2;
    if (a1.y < a2.y)
    {
        auto leftRect = a1;
        a1 = a2;
        a2 = leftRect;
    }
    if (b1.y < b2.y)
    {
        auto rightRect = b1;
        b1 = b2;
        b2 = rightRect;
    }

    cv::line(image, a1, b2, cv::Scalar(0, 255, 0), 3);
    cv::line(image, a2, b1, cv::Scalar(0, 255, 0), 3);
}

// 装甲板匹配
bool armorDetector::MatchArmor(const armorDetector::Light &right_light, const armorDetector::Light &left_light)
{
    double half_height = (right_light.light.size.height + left_light.light.size.height) / 4;
    if (std::min(right_light.area, left_light.area) * 4 < std::max(right_light.area, left_light.area)) // 面积差
        return false;
    else if (abs(left_light.light.center.y - right_light.light.center.y) > 0.8 * half_height) // 水平差
        return false;
    else if (abs(left_light.light.center.y - right_light.light.center.y) > max_height_difference_) // y轴差
        return false;
    else if (abs(left_light.light.center.x - right_light.light.center.x) < 15) // x轴差
        return false;
    else
        return true;
}

// 坐标获取
void armorDetector::GetArmorXYZ(armorDetector::Armor &target_Armor)
{
    cv::RotatedRect left = target_Armor.left_light.light;
    cv::RotatedRect right = target_Armor.right_light.light;
    std::vector<cv::Point2f> leftxy(4), rightxy(4);
    right.points(rightxy.data());
    left.points(leftxy.data());
    cv::Point2f a1 = (leftxy[1] + leftxy[2]) / 2;   // 左上
    cv::Point2f a2 = (leftxy[0] + leftxy[3]) / 2;   // 左下
    cv::Point2f b1 = (rightxy[1] + rightxy[2]) / 2; // 右上
    cv::Point2f b2 = (rightxy[0] + rightxy[3]) / 2; // 右下
    if (a1.y < a2.y)
    {
        auto leftRect = a1;
        a1 = a2;
        a2 = leftRect;
    }
    if (b1.y < b2.y)
    {
        auto rightRect = b1;
        b1 = b2;
        b2 = rightRect;
    }
    target_Armor.left_light.top = a1;
    target_Armor.left_light.bottom = a2;
    target_Armor.right_light.top = b1;
    target_Armor.right_light.bottom = b2;

    target_Armor.x = (a1.x + b2.x) / 2;
    target_Armor.y = (a1.y + b2.y) / 2;
}
