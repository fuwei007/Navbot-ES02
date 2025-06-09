#include "MahonyFilter.h"
#include <math.h>

// 构造函数实现
MahonyFilter::MahonyFilter(float kp, float ki) : twoKp(kp), twoKi(ki), 
    q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), 
    integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f) {}

// Mahony 滤波更新函数实现
void MahonyFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // 如果加速度计测量值为零，不进行滤波更新
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度计测量值
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 计算测量值与估计值之间的误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分误差
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 比例误差
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 四元数微分方程，使用 dt
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = q0;
    float qb = q1;
    float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// 获取四元数函数实现
void MahonyFilter::getQuaternion(float& q0_out, float& q1_out, float& q2_out, float& q3_out) {
    q0_out = q0;
    q1_out = q1;
    q2_out = q2;
    q3_out = q3;
}

// 四元数转欧拉角函数
void quaternionToEuler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw) {
    *roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    *pitch = asin(2 * (q0 * q2 - q3 * q1));
    *yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    // 将弧度转换为度
    *roll *= (180.0f / 3.1415926f);
    *pitch *= (180.0f / 3.1415926f);
    *yaw *= (180.0f / 3.1415926f);
}
