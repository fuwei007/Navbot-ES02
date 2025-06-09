#ifndef OllieFOCdrive_h
#define OllieFOCdrive_h

#include <Arduino.h>
#include "filter.h"
#include "touchscreen.h"


typedef struct
{
  unsigned char rxbuf[30];//接收数据的缓冲区
  unsigned char txbuf[30];
  unsigned char recstatu;//表示是否处于一个正在接收数据包的状态
  unsigned char count;//计数
  unsigned char packerflag;//是否接收到一个完整的数据包标志
  unsigned char dat;     
} Serial_t;


typedef struct
{
  // 步态参数
  //float stepSize;   // 每一步的角度变化
  
  float delayTime;  // 每一步的延迟时间
  //float cycleSteps; // 一个周期的时间
  float CurrentSteps; //当前步数
  float xt;       // 目标位置
  float xs1;       // 起始位置
  float xf1;       // 终点位置  
  float xs2;       // 起始位置
  float xf2;       // 终点位置
  float xs3;       // 起始位置
  float xf3;       // 终点位置  
  float xs4;       // 起始位置
  float xf4;       // 终点位置  
  float h;        // 最高位置  
  float H_fron;  //     
  float H_back;  //   
  float H_R;  //     
  float H_P;  //  
  float zs;       // 起始高度
  float lambda[2];   // λ 参数
  float sigma;
  float Ts;       // 周期

  float xo1;       // x1输出位置
  float zo1;        //输出最高位置

  float xo2;       // x2输出位置
  float zo2;        //输出最高位置

  float xo3;       // x3输出位置
  float zo3;        //输出最高位置

  float xo4;       // x4输出位置
  float zo4;        //输出最高位置  

  float MT[4];   //电机目标值

  float MotorVelocityF[4];   //电机速度


  uint8_t MotorMode;   //电机工作模式

  int Serial1HZ;
  int Serial1count;

  float BodyRoll4Wheel;
  float BodyPitching4Wheel;
  float BodyPitching4WheelT;
  float BodyPitching4WheelTF;
      
} body_t;


typedef union 
{
  struct 
  {
    float x;
    float y;
    float z;
  };
  float axis[3];
} Axis3f;

//姿态数据结构
typedef struct  
{
  Axis3f accf;       //滤波 加速度（G）
  Axis3f gyrof;      //滤波 陀螺仪（deg/s）  
  Axis3f acc;       //加速度（G）
  Axis3f gyro;      //陀螺仪（deg/s）  
  float roll;
  float pitch;
  float yaw;
  float temp;
} attitude_t;


typedef struct  
{
  Axis3f acc;       //加速度（G）
  Axis3f gyro;      //陀螺仪（deg/s） 
  float roll;
  float pitch;
  float yaw;

  float servo1;
  float servo2;
  float servo3;
  float servo4;

  
} zeroBias_t;

class MyPIDController {
  private:


  public:
  
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float deriv;
    float integral;  // 误差积分
    float previousError;  // 上一次的误差
    float iLimit;
    float outputLimit;
    float outP;
    float outI;
    float outD;    
    float output;
    float error;
    float enableDFilter;
    biquadFilter_t dFilter;  //
    float cutoffFreq;
  
    // 构造函数，初始化PID参数
    MyPIDController(float p, float i, float d, float iLimit, float outputLimit,float dt, float EnableDFilter, float CutoffFreq) {
      Kp = p;
      Ki = i;
      Kd = d;
      enableDFilter = EnableDFilter;
      integral = iLimit;
      previousError = outputLimit;//输出限幅
      cutoffFreq = CutoffFreq;
      
      if ((int)enableDFilter==1)
      {
        biquadFilterInitLPF(&dFilter, (1.0f/dt), (unsigned int)cutoffFreq);
      }      
      
    }

    // 计算PID输出的函数，参数包含误差和时间间隔dt
    float compute(float Error, float dt) {

      error = Error;
      // 计算误差积分
      integral += error * dt;

      //积分限幅
      if (iLimit != 0)
      {
        if(integral>iLimit)
          integral = iLimit;
        if(integral<(-iLimit))
          integral = -iLimit;
        
      }

      // 计算误差微分
      deriv = (error - previousError) / dt;
      if (enableDFilter==1)
      {
        deriv = biquadFilterApply(&dFilter, deriv);
      }

      
      outP = Kp * error;
      outI = Ki * integral;
      outD = Kd * deriv;


      // 计算PID输出
      output = outP + outI + outD;

      // 更新上一次的误差
      previousError = error;

      //输出限幅
      if (outputLimit != 0)
      {
        output = constrain(output, -outputLimit, outputLimit);
      }
  

      return output;
    }

    // 设置PID系数的函数
    void setPID(float p, float i, float d, float iLimit, float outputLimit,float dt, float EnableDFilter, float CutoffFreq) {
      Kp = p;
      Ki = i;
      Kd = d;
      enableDFilter = EnableDFilter;
      integral = iLimit;
      previousError = outputLimit;//输出限幅
      cutoffFreq = CutoffFreq;
      
      if ((int)enableDFilter)
      {
        biquadFilterInitLPF(&dFilter, (1.0f/dt), (unsigned int)cutoffFreq);
      }    
    }
};




#endif
