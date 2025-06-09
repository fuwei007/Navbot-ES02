#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

// 定义默认的舵机引脚
const int DEFAULT_SERVO_1_PIN = 11;
const int DEFAULT_SERVO_2_PIN = 12;
const int DEFAULT_SERVO_3_PIN = 21;
const int DEFAULT_SERVO_4_PIN = 14;

// 定义PWM信号的频率，舵机通常使用50Hz（20ms周期）
const int PWM_FREQUENCY = 50;
// 定义PWM信号的分辨率，设置为12位
const int PWM_RESOLUTION = 12;

// 定义舵机的最小和最大脉冲宽度对应的占空比
const float MIN_PULSE_WIDTH_DUTY = 102.4; // 对应0.5ms
const float MAX_PULSE_WIDTH_DUTY = 512;   // 对应2.5ms

class ServoControl {
public:
    // 构造函数，允许用户传入自定义的引脚
    ServoControl(int servo1Pin = DEFAULT_SERVO_1_PIN, int servo2Pin = DEFAULT_SERVO_2_PIN,
                 int servo3Pin = DEFAULT_SERVO_3_PIN, int servo4Pin = DEFAULT_SERVO_4_PIN);
    // 初始化舵机
    void initialize();
    // 设置四个舵机的角度
    void setServosAngle(int direction1, int angle1, int direction2, int angle2,
                        int direction3, int angle3, int direction4, int angle4,float dt_ms);

private:
    int servo1Pin;
    int servo2Pin;
    int servo3Pin;
    int servo4Pin;
    // 根据设置的方向和角度计算舵机的PWM占空比
    int calculateServoPwmDutyCycle(int direction, int angle);
};

#endif
