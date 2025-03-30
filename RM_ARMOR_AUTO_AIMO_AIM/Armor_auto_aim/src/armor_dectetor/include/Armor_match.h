// Author:xpoying
// 头文件 装甲板结构体，装甲板识别类，标记装甲板的函数声明
#ifndef ARMOR_MATCH_H
#define ARMOR_MATCH_H

//opencv
#include "opencv2/opencv.hpp"
#include <opencv2/video/tracking.hpp>

//std
#include <iostream>
#include <vector>

//#include <tf2/convert.h>
//#include <tf2/LinearMath/Matrix3x3.h>

//装甲板识别
class armorDetector
{
public:
   //灯条
   struct Light
    {
        cv::RotatedRect light;    
        double area;
        double angle;
        cv::Point2f top;
        cv::Point2f bottom;
    };
  
    //装甲板
    struct Armor
    {
        armorDetector::Light left_light;
        armorDetector::Light right_light;
        float armor_angle;
        float armor_speed;
        //单位:mm
        static constexpr float ARMOR_WIDTH = 230;
        static constexpr float ARMOR_HEIGHT = 127;
        //相机坐标
        float position_x;
        float position_y;
        float position_z;
        //图像坐标
        float x;
        float y;
        // 预测坐标
        float pre_x = 0;
        float pre_y = 0;
        //到图像中心的距离
        float distance_to_center;
        //tf2::Quaternion tf2_q;
    }last_armor,new_armor;       //前一个时刻的装甲板与当前时刻的装甲板

        // 相机内参
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1034.174789, 0.000000, 638.103833,
                                 0.000000, 1034.712789, 494.576371,
                                 0.000000, 0.000000, 1.000000);
        // 畸变矩阵
        cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << -0.100831, 0.094629, -0.000572, 0.000629, 0.000000);

        // 校正矩阵
        cv::Mat rectification_matrix = (cv::Mat_<double>(3, 3) << 1.000000, 0.000000, 0.000000,
                                        0.000000, 1.000000, 0.000000,
                                        0.000000, 0.000000, 1.000000);
        // 投影矩阵
        cv::Mat projection_matrix = (cv::Mat_<double>(3, 4) << 1008.22804, 0.000000, 638.99992,
                                     0.000000, 0.000000, 1014.43577,
                                     493.63869, 0.000000, 0.000000,
                                     0.000000, 1.000000, 0.000000);
    
    explicit armorDetector(double max_height_difference, double max_length_width_ratio, double min_length_width_ratio);
    void ProcessImage(const cv::Mat &image, cv::Mat &edges, const float threshold);
    void DrawArmor(const cv::Mat &image, Armor &target_Armor);
    bool IsLight(cv::RotatedRect &minRects);
    bool MatchArmor(const armorDetector::Light &right_light, const armorDetector::Light &left_light);
    void GetArmorXYZ(armorDetector::Armor &armor_);
    bool IsSameArmor();
    bool update_armor_ = 0;
private:
    double max_height_difference_;
    double max_length_width_ratio_;
    double min_length_width_ratio_;
};

#endif
