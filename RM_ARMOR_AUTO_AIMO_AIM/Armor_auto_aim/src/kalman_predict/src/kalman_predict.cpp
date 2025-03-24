#include "Kalman_predict.h"

//卡尔曼滤波实现   主要使用cv::KalmanFilter kalman来计算

//卡尔曼滤波初始化
void kalman_armor::Init(int DP, int MP, int CP)
{
    statePre = cv::Mat::zeros(DP, 1, CV_32F); // 预测状态向量 x'(k)
    stateOpt = cv::Mat::zeros(DP, 1, CV_32F); // 修正状态向量x(k)
    kalman.init(DP, MP, CP, CV_32F);
}

//机动追踪模型初始化参数
void kalman_armor::SingerInit(float alpha, float dt, float p, float k, float r)
{
    singer[0] = alpha;
    singer[1] = dt;
    singer[2] = p;
    singer[3] = k;
    singer[4] = r;
}

////机动追踪模型初始化矩阵
void kalman_armor::KfinitSinger()
{
    // 初始化卡尔曼滤波器，状态维度为3，测量维度为3，控制维度为1
    float alpha = singer[0]; // 参数 alpha
    float dt = singer[1];    // 时间步长 dt

    // 初始化状态转移矩阵 A
    kalman.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
                               0, 1, (1 - exp(-alpha * dt)) / alpha,
                               0, 0, exp(-alpha * dt));

    // 初始化测量矩阵 H
    kalman.measurementMatrix = (cv::Mat_<float>(1, 3) << 1, 0, 0);
    // 初始化控制矩阵 B
    kalman.controlMatrix = (cv::Mat_<float>(3, 1) << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                            dt - (1 - exp(-alpha * dt) / alpha),
                            1 - exp(-alpha * dt));

    // 初始化后验误差协方差矩阵 P(k)
    float p = singer[2];
    kalman.errorCovPost = (cv::Mat_<float>(3, 3) << p, 0, 0,
                           0, p, 0,
                           0, 0, p);

    // 计算过程噪声协方差矩阵 Q
    float q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
    float q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
    float q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
    float q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
    float q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
    float q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));

    float k = singer[3];
    kalman.processNoiseCov = (cv::Mat_<float>(3, 3) << k * alpha * q11, k * alpha * q12, k * alpha * q13,
                              k * alpha * q12, k * alpha * q22, k * alpha * q23,
                              k * alpha * q13, k * alpha * q23, k * alpha * q33);

    // 初始化测量噪声协方差矩阵 R
    float r = singer[4];
    kalman.measurementNoiseCov = r * cv::Mat::eye(1, 1, CV_32F);
}

//卡尔曼预测
void kalman_armor::KalmanPredict()
{
    statePre = kalman.predict();
}

//更新卡尔曼滤波
void kalman_armor::KalmanUpdata(cv::Mat measureMat_last)
{
    stateOpt = kalman.correct(measureMat_last);
}

//----------------------------------------------------------------------------------------------------------
//初始化CA
void kalman_armor::kfinit_uniform()
{
    float dt = 0.8f;                 // 时间步长 dt
    float process_noise = 1e-6f;     // 过程噪声
    float measurement_noise = 1e-2f; // 测量噪声
    float error_covariance = 1e-2f;  // 初始误差协方差
    // 使用匀加速模型
    kalman.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, dt, 0.5f * dt * dt,
                               0, 1, dt,
                               0, 0, 1);

    kalman.measurementMatrix = (cv::Mat_<float>(1, 3) << 1, 0, 0);

    // 初始化后验误差协方差矩阵 P(k)
    kalman.errorCovPost = cv::Mat::eye(3, 3, CV_32F) * error_covariance;

    // 初始化过程噪声协方差矩阵 Q
    kalman.processNoiseCov = cv::Mat::eye(3, 3, CV_32F) * process_noise;

    // 初始化测量噪声协方差矩阵 R
    kalman.measurementNoiseCov = measurement_noise * cv::Mat::eye(1, 1, CV_32F);
}

//判断是否是同一个装甲板
bool kalman_armor::IsSameArmor()
{
    if (abs(last_armor.position_y - new_armor.position_y) < 0.3)
        return true;
    else
        return false;
}