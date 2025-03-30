#include "Kalman_Filter.h"

void Kalman_Filter::Init(int DP, int MP, int CP)
{
    assert(DP > 0 && MP > 0);
    CP = std::max(CP, 0);

    this->DP = DP;
    this->MP = MP;
    this->CP = CP;

    statePre = MatrixXf::Zero(DP, 1);             // predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    stateOpt = MatrixXf::Zero(DP, 1);             // corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    transMat = MatrixXf::Zero(DP, DP);            // A
    processNoiseCov = MatrixXf::Identity(DP, DP); // Q
    measureMat = MatrixXf::Zero(MP, DP);          // H
    measurement = MatrixXf::Zero(MP, 1);          // Z
    measureNoiseCov = MatrixXf::Identity(MP, MP); // R
    errorCovpre = MatrixXf::Zero(DP, DP);         //(P'(k)): P'(k)=A*P(k-1)*At + Q)
    errorCovOpt = MatrixXf::Zero(DP, DP);         //(P(k)) : P(k) = (I - K(k)*H)*P'(k)
    Kgain = MatrixXf::Zero(DP, MP);               //(K(k)) : K(k) = P'(k)*Ht*inv(H*P'(k)*Ht + R)

    //
    if (CP > 0)
        contrMat = MatrixXf::Zero(DP, CP);
}

void Kalman_Filter::KalmanCA()
{
    TI = 1;
    processNoiseCov << 1e-6f, 0, 0,
        0, 1e-6f, 0,
        0, 0, 1e-6f;

    measureNoiseCov = MatrixXf::Identity(1, 1) * 1e-2f;

    //
    transMat << 1, TI, TI * TI * 0.5,
        0, 1, TI,
        0, 0, 1;

    // 初始化测量矩阵
    measureMat << 1, 0, 0;

    // 初始化误差协方差矩阵（后验协方差）
    errorCovOpt << 1e-2f, 0, 0,
        0, 1e-2f, 0,
        0, 0, 1e-2f;
}

void Kalman_Filter::Predict()
{

    // 预测状态：statePre = A * stateOpt
    if (CP > 0)
        statePre = transMat * stateOpt + contrMat * stateOpt(2, 0);
    else
        statePre = transMat * stateOpt;

    // 预测误差协方差：errorCovpre = A * errorCovOpt * A^T + Q
    errorCovpre = transMat * errorCovOpt * transMat.transpose() + processNoiseCov;
}

void Kalman_Filter::Update()
{

    Kgain = errorCovpre * measureMat.transpose() * (measureMat * errorCovpre * measureMat.transpose() + measureNoiseCov).inverse();

    stateOpt = statePre + Kgain * (measurement - measureMat * statePre);

    errorCovOpt = (MatrixXf::Identity(DP, DP) - Kgain * measureMat) * errorCovpre;
};

void Kalman_Filter::SingerInit(float alpha, float dt, float p, float k, float r)
{
    // 初始化卡尔曼滤波器，状态维度为3，测量维度为3，控制维度为1

    // 初始化状态转移矩阵 A
    transMat << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
        0, 1, (1 - exp(-alpha * dt)) / alpha,
        0, 0, exp(-alpha * dt);

    // 初始化测量矩阵 H
    measureMat << 1, 0, 0;
    // 初始化控制矩阵 B
    contrMat << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
        dt - (1 - exp(-alpha * dt) / alpha),
        1 - exp(-alpha * dt);

    // 初始化后验误差协方差矩阵 P(k)
    errorCovOpt << p, 0, 0,
        0, p, 0,
        0, 0, p;

    // 计算过程噪声协方差矩阵 Q
    float q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
    float q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
    float q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
    float q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
    float q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
    float q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));

    processNoiseCov << k * alpha * q11, k * alpha * q12, k * alpha * q13,
        k * alpha * q12, k * alpha * q22, k * alpha * q23,
        k * alpha * q13, k * alpha * q23, k * alpha * q33;

    // 初始化测量噪声协方差矩阵 R
    measureNoiseCov = r * MatrixXf::Identity(1, 1);
}