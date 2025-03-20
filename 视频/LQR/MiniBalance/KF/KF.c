#include "KF.h"
/**************************************************************************
Function: 卡尔曼滤波
Input   : 角速度，加速度
Output  : 无
**************************************************************************/
float KF_X(float acce_Y, float acce_Z, float gyro_X) // 输入量：Y轴加速度，Z轴加速度，X轴角速度。
{
    static float x_hat[2][1] = {0}; // 后验估计
    static float x_hat_minus[2][1] = {0}; // 先验估计
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // 后验误差协方差矩阵
    static float p_hat_minus[2][2] = {0}; // 先验误差协方差矩阵
    static float K[2][1] = {0}; // 卡尔曼增益
    const float Ts = 0.005; // 采样间隔(5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_X}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A矩阵
    float B[2][1] = {{Ts}, {0}}; // B矩阵
    float C[1][2] = {{1, 0}};// C矩阵
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // 过程噪声
    float R[1][1] = {{1e-4}}; // 测量噪声
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // A矩阵的转置
    float C_T[2][1] = {{1}, {0}}; // C矩阵的转置
    float temp_1[2][1] = {0}; // 用以存储中间计算结果
    float temp_2[2][1] = {0}; // 用以存储中间计算结果
    float temp_3[2][2] = {0}; // 用以存储中间计算结果
    float temp_4[2][2] = {0}; // 用以存储中间计算结果
    float temp_5[1][2] = {0}; // 用以存储中间计算结果
    float temp_6[1][1] = {0}; // 用以存储中间计算结果
    float y = atan2(-acce_Y, acce_Z); // 利用加速度计算角度
    // 预测部分
    // 先验估计公式
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // 先验误差协方差公式
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    // 校正部分
    // 卡尔曼增益公式
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    // 后验估计公式
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // 更新误差协方差公式
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // 返回值
    return x_hat[0][0];
}

/**************************************************************************
Function: 卡尔曼滤波
Input   : 角速度，加速度
Output  : 无
**************************************************************************/
float KF_Y(float acce_X, float acce_Z, float gyro_Y) // 输入量：X轴加速度，Z轴加速度，Y轴角速度。
{
    static float x_hat[2][1] = {0}; // 后验估计
    static float x_hat_minus[2][1] = {0}; // 先验估计
    static float p_hat[2][2] = {{1, 0}, {0, 1}}; // 后验误差协方差矩阵
    static float p_hat_minus[2][2] = {0}; // 先验误差协方差矩阵
    static float K[2][1] = {0}; // 卡尔曼增益
    const float Ts = 0.005; // 采样间隔(5ms)
    const float I[2][2] = {{1, 0}, {0, 1}};
    float u[1][1] = {{gyro_Y}};
    float A[2][2] = {{1, -Ts}, {0, 1}}; // A矩阵
    float B[2][1] = {{Ts}, {0}}; // B矩阵
    float C[1][2] = {{1, 0}};// C矩阵
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // 过程噪声
    float R[1][1] = {{1e-4}}; // 测量噪声
    float A_T[2][2] = {{1, 0}, {-Ts, 1}}; // A矩阵的转置
    float C_T[2][1] = {{1}, {0}}; // C矩阵的转置
    float temp_1[2][1] = {0}; // 用以存储中间计算结果
    float temp_2[2][1] = {0}; // 用以存储中间计算结果
    float temp_3[2][2] = {0}; // 用以存储中间计算结果
    float temp_4[2][2] = {0}; // 用以存储中间计算结果
    float temp_5[1][2] = {0}; // 用以存储中间计算结果
    float temp_6[1][1] = {0}; // 用以存储中间计算结果
    float y = atan2(-acce_X, acce_Z); // 利用加速度计算角度
    // 预测部分
    // 先验估计公式
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    // 先验误差协方差公式
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    // 校正部分
    // 卡尔曼增益公式
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    // 后验估计公式
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    // 更新误差协方差公式
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    // 返回值
    return x_hat[0][0];
}

/**************************************************************************
Function: 矩阵乘法
Input   : 需要相乘的两个矩阵以及它们的尺寸
Output  : 相乘后的矩阵
**************************************************************************/
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col])
{
    if (A_col == B_row)
    {
        for (int i = 0; i < A_row; i++)
        {
            for (int j = 0; j < B_col; j++)
            {
                C[i][j] = 0; // 初始化
                for (int k = 0; k < A_col; k++)
                {
                    C[i][j] += A[i][k]*B[k][j];
                }
            }
        }
    }
    else
    {
        printf("错误：矩阵的尺寸不对！");
    }
}

















