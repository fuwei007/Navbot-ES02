#include "ServoControl.h"


// Constructor, initialize servo pins
ServoControl::ServoControl(int servo1Pin, int servo2Pin, int servo3Pin, int servo4Pin)
    : servo1Pin(servo1Pin), servo2Pin(servo2Pin), servo3Pin(servo3Pin), servo4Pin(servo4Pin) {}

// Initialize PWM channels for servos
void ServoControl::initialize() {
    ledcAttach(servo1Pin, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(servo2Pin, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(servo3Pin, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(servo4Pin, PWM_FREQUENCY, PWM_RESOLUTION);

    // Set initial angle of four servos to 0 degrees
    setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    delay(22);
    setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    delay(22);
    setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    delay(22);        
}

// Set angles for four servos
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

// Calculate servo PWM duty cycle based on direction and angle settings
int ServoControl::calculateServoPwmDutyCycle(int direction, int angle) {
    // Adjust angle based on direction
    if (direction == -1) {
        angle = -angle;
    }

    // Ensure angle is within valid range of -90 to 90 degrees
    if (angle < -90) {
        angle = -90;
    }
    if (angle > 90) {
        angle = 90;
    }

    // Map angle from -90 to 90 degrees to 0 to 180 degrees
    angle = map(angle, -90, 90, 0, 180);

    // Ensure mapped angle is within valid range of 0 to 180 degrees
    if (angle < 0) {
        angle = 0;
    }
    if (angle > 180) {
        angle = 180;
    }

    // Calculate PWM duty cycle for given angle through linear mapping
    return (int)(((MAX_PULSE_WIDTH_DUTY - MIN_PULSE_WIDTH_DUTY) / 180) * angle + MIN_PULSE_WIDTH_DUTY);
}
