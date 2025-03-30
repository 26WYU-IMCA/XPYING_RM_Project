#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// std
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <vector>

using namespace Eigen;

class Kalman_Filter
{
public:
    void Init(int DP, int MP, int CP);                                 // 卡尔曼滤波初始化
    void KalmanCA();                                                   // 匀加速度模型初始化
    void Predict();                                                    // 预测
    void Update();                                                     // 更新
    void SingerInit(float alpha, float dt, float p, float k, float r); // 机动追踪模型初始化
    int DP;
    int MP;
    int CP;
    int TI;
    float singer[5];          // 机动追踪模型参数
    MatrixXf statePre;        // 预测状态向量 x'(k)   先验估计
    MatrixXf stateOpt;        // 修正状态向量x(k)     后验估计
    MatrixXf transMat;        // 状态转移矩阵A
    MatrixXf measureMat;      // 测量矩阵H
    MatrixXf measurement;     // 测量值Z
    MatrixXf processNoiseCov; // 过程噪声协方差Q
    MatrixXf measureNoiseCov; // 测量噪声协方差R
    MatrixXf errorCovpre;     // 先验误差协方差P'(k)
    MatrixXf errorCovOpt;     // 后验误差协方差P(k)
    MatrixXf Kgain;           // 卡尔曼增益
    MatrixXf contrMat;        // 控制矩阵B
};

#endif