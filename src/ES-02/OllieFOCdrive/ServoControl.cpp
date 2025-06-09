#include "ServoControl.h"


// 构造函数，初始化舵机引脚
ServoControl::ServoControl(int servo1Pin, int servo2Pin, int servo3Pin, int servo4Pin)
    : servo1Pin(servo1Pin), servo2Pin(servo2Pin), servo3Pin(servo3Pin), servo4Pin(servo4Pin) {}

// 初始化舵机的PWM通道
void ServoControl::initialize() {
    ledcAttach(servo1Pin, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(servo2Pin, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(servo3Pin, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(servo4Pin, PWM_FREQUENCY, PWM_RESOLUTION);

    // 将四个舵机的初始角度设置为 0 度
    setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    delay(22);
    setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    delay(22);
    setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    delay(22);        
}

// 设置四个舵机的角度
void ServoControl::setServosAngle(int direction1, int angle1, int direction2, int angle2,
                                  int direction3, int angle3, int direction4, int angle4,float dt_ms) {

    static unsigned long now_ms = millis();
    static unsigned long now_ms1 = now_ms;
    static float dt = 0;
    now_ms = millis();
    dt = (now_ms - now_ms1) / 1000.0f;
    if((dt>=dt_ms)||((int)dt_ms==1))//
    {
      now_ms1 = now_ms;
      //Serial.println(dt,6);
      ledcWrite(servo1Pin, calculateServoPwmDutyCycle(direction1, angle1));
      ledcWrite(servo2Pin, calculateServoPwmDutyCycle(direction2, angle2));
      ledcWrite(servo3Pin, calculateServoPwmDutyCycle(direction3, angle3));
      ledcWrite(servo4Pin, calculateServoPwmDutyCycle(direction4, angle4));      
    }                                
}

// 根据设置的方向和角度计算舵机的PWM占空比
int ServoControl::calculateServoPwmDutyCycle(int direction, int angle) {
    // 根据方向调整角度
    if (direction == -1) {
        angle = -angle;
    }

    // 确保角度在 -90 到 90 度的有效范围内
    if (angle < -90) {
        angle = -90;
    }
    if (angle > 90) {
        angle = 90;
    }

    // 将 -90 到 90 度的角度映射到 0 到 180 度
    angle = map(angle, -90, 90, 0, 180);

    // 确保映射后的角度在 0 到 180 度的有效范围内
    if (angle < 0) {
        angle = 0;
    }
    if (angle > 180) {
        angle = 180;
    }

    // 通过线性映射计算出给定角度对应的PWM占空比
    return (int)(((MAX_PULSE_WIDTH_DUTY - MIN_PULSE_WIDTH_DUTY) / 180) * angle + MIN_PULSE_WIDTH_DUTY);
}
