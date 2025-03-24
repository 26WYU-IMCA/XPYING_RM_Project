#ifndef KALMAN_PREDICT_H
#define KALMAN_PREDICT_H

// opemcv
#include "opencv2/opencv.hpp"

// std
#include <iostream>
#include <vector>
//#include <Eigen/Dense>

// 卡尔曼滤波类
class kalman_armor
{
public:
    // 装甲板
    struct Armorgg
    {
        float armor_angle;
        float armor_speed;
        // 单位:mm
        static constexpr float ARMOR_WIDTH = 230;
        static constexpr float ARMOR_HEIGHT = 127;
        // 相机坐标
        float position_x;
        float position_y;
        float position_z;
        // 图像坐标
        float x;
        float y;
        // 预测坐标
        float pre_x = 0;
        float pre_y = 0;
        // 到图像中心的距离
        float distance_to_center;
        // tf2::Quaternion tf2_q;
    } last_armor, new_armor; // 前一个时刻的装甲板与当前时刻的装甲板

    // 判断是否是同一个目标
    bool IsSameArmor();
    bool update_armor_ = 0;

    void Init(int DP_, int MP_, int CP_);                               // 卡尔曼滤波初始化
    void SingerInit(float alpha, float dt, float p, float k, float r); // 机动追踪模型初始化
    void KalmanPredict();                                              // 卡尔曼滤波预测
    void KalmanUpdata(cv::Mat measureMat_last);                        // 卡尔曼滤波更新预测值
    float singer[5];                                                    // 机动追踪模型参数
    int DP;
    int MP;
    int CP;
    cv::Mat statePre;         // 预测状态向量 x'(k)
    cv::Mat stateOpt;         // 修正状态向量x(k)
    cv::Mat transMat;         // 状态转移矩阵A
    cv::Mat measureMat;       // 测量矩阵H
    cv::Mat processNoiseCov;  // 过程噪声协方差Q
    cv::Mat measureNosiseCov; // 测量噪声协方差R
    cv::Mat errorCovpre;      // 先验误差协方差P'(k)
    cv::Mat errorCovOpt;      // 后验误差协方差P(k)
    cv::Mat kgain;            // 卡尔曼增益
    cv::Mat contrMat;         // 控制矩阵B
    cv::KalmanFilter kalman;  // 卡尔曼滤波

    void kfinit_uniform(); // 匀加速模型
    void KfinitSinger();  // 机动追踪模型
};

// class Kalman
// {
// public:
//     void init(int DP,int MP,int CP);   // 卡尔曼滤波初始化
    
// private:
//     int DP;
//     int MP;
//     int CP;
//     Eigen::MatrixXf statePre;         // 预测状态向量 x'(k)
//     Eigen::MatrixXf stateOpt;         // 修正状态向量x(k)
//     Eigen::MatrixXf transMat;         // 状态转移矩阵A
//     Eigen::MatrixXf measureMat;       // 测量矩阵H
//     Eigen::MatrixXf processNoiseCov;  // 过程噪声协方差Q
//     Eigen::MatrixXf measureNosiseCov; // 测量噪声协方差R
//     Eigen::MatrixXf errorCovpre;      // 先验误差协方差P'(k)
//     Eigen::MatrixXf errorCovOpt;      // 后验误差协方差P(k)
//     Eigen::MatrixXf kgain;            // 卡尔曼增益
//     Eigen::MatrixXf contrMat;         // 控制矩阵B
// };
#endif