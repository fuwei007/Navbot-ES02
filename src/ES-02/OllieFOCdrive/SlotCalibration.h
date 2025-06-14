#ifndef _SlotCalibration_H_
#define _SlotCalibration_H_
#include <Arduino.h>
#include <SimpleFOC.h>

#define TorqueSamples 720 // Number of samples
#define AngleResolutionRatio (_2PI/TorqueSamples) // Angular resolution
#define SlotVelocityLimit 0.03f // Adjust based on actual noise
#define SlotPositionErrorLimit 0.001f // Adjust based on actual noise, position error limit in radians

extern int Slot_calibration_mark; // Calibration completion flag
extern float Motor1_Current_sp_data[TorqueSamples];
extern float Motor2_Current_sp_data[TorqueSamples];

void CalibrationCurrentSp(float Position_estimation,float Velocity_estimation,BLDCMotor *Motor);





 
#endif
