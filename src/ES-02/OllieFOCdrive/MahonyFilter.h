// MahonyFilter.h
#ifndef MAHONYFILTER_H
#define MAHONYFILTER_H

class MahonyFilter {
private:

    float q0, q1, q2, q3; // 四元数初始值
    float integralFBx, integralFBy, integralFBz; // 积分误差项

public:
    float twoKp;    // 比例增益
    float twoKi;    // 积分增益
    
    // 构造函数，初始化参数
    MahonyFilter(float kp = 0.400f, float ki = 0.001f);

    // Mahony 滤波更新函数
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    // 获取四元数
    void getQuaternion(float& q0_out, float& q1_out, float& q2_out, float& q3_out);
};


// 四元数转欧拉角函数
void quaternionToEuler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

#endif
