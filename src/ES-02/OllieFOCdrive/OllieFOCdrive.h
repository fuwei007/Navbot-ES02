#ifndef OllieFOCdrive_h
#define OllieFOCdrive_h

#include <Arduino.h>
#include "filter.h"
#include "touchscreen.h"

#define SWITCHING_PATTERN_TWO_WHEEL_MODE 0  // Self-balancing two-wheel mode
#define SWITCHING_PATTERN_FOUR_WHEEL_MODE 1 // Four-legged walking mode

#define MASTER_SLAVE_SELECTION_SLAVE 0  // The controller is a slave
#define MASTER_SLAVE_SELECTION_MASTER 1 // The controller is the master

#define ADJUSTMENT_PARAM_BALANCE_SPEED_YAW_ROLL 0
#define ADJUSTMENT_PARAM_BALL_PUSHING 1 // For tuning main balance and movement PID loops

#define SWITCH_USER_MODE_VIEW_ENCODER 0     // View encoder position and direction
#define SWITCH_USER_MODE_SAMPLE_TORQUE_M1 1 // Sample motor 1 torque compensation data
#define SWITCH_USER_MODE_SAMPLE_TORQUE_M2 2 // Sample motor 2 torque compensation data
#define SWITCH_USER_MODE_TORQUE_MODE 3      // Direct torque control mode
#define SWITCH_USER_MODE_SPEED_MODE 4       // Speed control mode
#define SWITCH_USER_MODE_ANGLE_MODE 5       // Angle control mode

#define TORQUE_COMPENSATION_OFF 0 // Torque compensation is off
#define TORQUE_COMPENSATION_ON 1  // Torque compensation is on

#define COMMUNICATION_OBJECT_TWO_OR_FOUR_WHEEL_BALANCE 0 // Two-wheel balance mode
#define COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO 1          // SimpleFOC Studio host computer
#define COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS 2       // Control dual motors
#define COMMUNICATION_OBJECT_SAMPLE_TORQUE_DATA 3        // Sample torque data

#define ROBOT_TUMBLE_NO 0  // Robot is not tumbling
#define ROBOT_TUMBLE_YES 1 // Robot is tumbling

#define SENSOR_SWITCH_SPI 1
#define SENSOR_SWITCH_IIC_AS5600 2

#define CURRENT_LOOP_OFF 0 // Current loop is off
#define CURRENT_LOOP_ON 1  // Current loop is on

#define REMOTE_CONTROL_PID_GAINS_MODE_OFF 0
#define REMOTE_CONTROL_PID_GAINS_MODE_ON_WITHOUT_TOUCH 1
#define REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH 2

#define REMOTE_CONTROL_PM_POSTURE_MODE 0 // Posture control mode
#define REMOTE_CONTROL_PM_MARK_MODE 1    // Mark control mode

#define REMOTE_CONTROL_ROLL_MODE_MANUAL 0
#define REMOTE_CONTROL_ROLL_MODE_AUTO 1

#define REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT 0
#define REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST 1 // Pitching adjustment mode
#define REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE 2      // Ball poise mode

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
