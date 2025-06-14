#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

// Define default servo pins
const int DEFAULT_SERVO_1_PIN = 11;
const int DEFAULT_SERVO_2_PIN = 12;
const int DEFAULT_SERVO_3_PIN = 21;
const int DEFAULT_SERVO_4_PIN = 14;

// Define PWM signal frequency, servos typically use 50Hz (20ms period)
const int PWM_FREQUENCY = 50;
// Define PWM signal resolution, set to 12 bits
const int PWM_RESOLUTION = 12;

// Define duty cycle corresponding to minimum and maximum pulse width for servos
const float MIN_PULSE_WIDTH_DUTY = 102.4; // Corresponds to 0.5ms
const float MAX_PULSE_WIDTH_DUTY = 512;   // Corresponds to 2.5ms

class ServoControl {
public:
    // Constructor, allows users to input custom pins
    ServoControl(int servo1Pin = DEFAULT_SERVO_1_PIN, int servo2Pin = DEFAULT_SERVO_2_PIN,
                 int servo3Pin = DEFAULT_SERVO_3_PIN, int servo4Pin = DEFAULT_SERVO_4_PIN);
    // Initialize servos
    void initialize();
    // Set angles for four servos
    void setServosAngle(int direction1, int angle1, int direction2, int angle2,
                        int direction3, int angle3, int direction4, int angle4,float dt_ms);

private:
    int servo1Pin;
    int servo2Pin;
    int servo3Pin;
    int servo4Pin;
    // Calculate servo PWM duty cycle based on direction and angle settings
    int calculateServoPwmDutyCycle(int direction, int angle);
};

#endif
