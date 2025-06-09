#ifndef _SlotCalibration_H_
#define _SlotCalibration_H_
#include <Arduino.h>
#include <SimpleFOC.h>

#define TorqueSamples 720 //采样数
#define AngleResolutionRatio (_2PI/TorqueSamples) //弧度分辨率
#define SlotVelocityLimit 0.03f //根据实际噪声调整
#define SlotPositionErrorLimit 0.001f //根据实际噪声调整 位置误差限制 弧度

extern int Slot_calibration_mark;//标定完成标志
extern float Motor1_Current_sp_data[TorqueSamples];
extern float Motor2_Current_sp_data[TorqueSamples];

void CalibrationCurrentSp(float Position_estimation,float Velocity_estimation,BLDCMotor *Motor);





 
#endif
