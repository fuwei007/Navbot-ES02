#ifndef OllieFOCdrive_h
#define OllieFOCdrive_h

#include <Arduino.h>
#include "filter.h"
#include "touchscreen.h"


typedef struct
{
  unsigned char rxbuf[30];//Receive data buffer
  unsigned char txbuf[30];
  unsigned char recstatu;//Indicates whether in a state of receiving data packet
  unsigned char count;//Counter
  unsigned char packerflag;//Flag indicating whether a complete data packet is received
  unsigned char dat;     
} Serial_t;


typedef struct
{
  // Gait parameters
  //float stepSize;   // Angle change per step
  
  float delayTime;  // Delay time per step
  //float cycleSteps; // Time for one cycle
  float CurrentSteps; //Current step count
  float xt;       // Target position
  float xs1;       // Start position
  float xf1;       // End position  
  float xs2;       // Start position
  float xf2;       // End position
  float xs3;       // Start position
  float xf3;       // End position  
  float xs4;       // Start position
  float xf4;       // End position  
  float h;        // Maximum height  
  float H_fron;  //     
  float H_back;  //   
  float H_R;  //     
  float H_P;  //  
  float zs;       // Start height
  float lambda[2];   // λ parameter
  float sigma;
  float Ts;       // Period

  float xo1;       // x1 output position
  float zo1;        //Output maximum height

  float xo2;       // x2 output position
  float zo2;        //Output maximum height

  float xo3;       // x3 output position
  float zo3;        //Output maximum height

  float xo4;       // x4 output position
  float zo4;        //Output maximum height  

  float MT[4];   //Motor target values

  float MotorVelocityF[4];   //Motor velocity


  uint8_t MotorMode;   //Motor working mode

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

//Attitude data structure
typedef struct  
{
  Axis3f accf;       //Filtered acceleration (G)
  Axis3f gyrof;      //Filtered gyroscope (deg/s)  
  Axis3f acc;       //Acceleration (G)
  Axis3f gyro;      //Gyroscope (deg/s)  
  float roll;
  float pitch;
  float yaw;
  float temp;
} attitude_t;


typedef struct  
{
  Axis3f acc;       //Acceleration (G)
  Axis3f gyro;      //Gyroscope (deg/s) 
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
  
    float Kp;  // Proportional coefficient
    float Ki;  // Integral coefficient
    float Kd;  // Derivative coefficient
    float deriv;
    float integral;  // Error integral
    float previousError;  // Previous error
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
  
    // Constructor, initialize PID parameters
    MyPIDController(float p, float i, float d, float iLimit, float outputLimit,float dt, float EnableDFilter, float CutoffFreq) {
      Kp = p;
      Ki = i;
      Kd = d;
      enableDFilter = EnableDFilter;
      integral = iLimit;
      previousError = outputLimit;//Output limit
      cutoffFreq = CutoffFreq;
      
      if ((int)enableDFilter==1)
      {
        biquadFilterInitLPF(&dFilter, (1.0f/dt), (unsigned int)cutoffFreq);
      }      
      
    }

    // Function to calculate PID output, parameters include error and time interval dt
    float compute(float Error, float dt) {

      error = Error;
      // Calculate error integral
      integral += error * dt;

      //Integral limit
      if (iLimit != 0)
      {
        if(integral>iLimit)
          integral = iLimit;
        if(integral<(-iLimit))
          integral = -iLimit;
        
      }

      // Calculate error derivative
      deriv = (error - previousError) / dt;
      if (enableDFilter==1)
      {
        deriv = biquadFilterApply(&dFilter, deriv);
      }

      
      outP = Kp * error;
      outI = Ki * integral;
      outD = Kd * deriv;


      // Calculate PID output
      output = outP + outI + outD;

      // Update previous error
      previousError = error;

      //Output limit
      if (outputLimit != 0)
      {
        output = constrain(output, -outputLimit, outputLimit);
      }
  

      return output;
    }

    // Function to set PID coefficients
    void setPID(float p, float i, float d, float iLimit, float outputLimit,float dt, float EnableDFilter, float CutoffFreq) {
      Kp = p;
      Ki = i;
      Kd = d;
      enableDFilter = EnableDFilter;
      integral = iLimit;
      previousError = outputLimit;//Output limit
      cutoffFreq = CutoffFreq;
      
      if ((int)enableDFilter)
      {
        biquadFilterInitLPF(&dFilter, (1.0f/dt), (unsigned int)cutoffFreq);
      }    
    }
};




#endif
