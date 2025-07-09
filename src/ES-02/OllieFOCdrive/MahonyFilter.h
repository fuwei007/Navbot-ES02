// MahonyFilter.h
#ifndef MAHONYFILTER_H
#define MAHONYFILTER_H

class MahonyFilter {
private:

    float q0, q1, q2, q3; // Quaternion initial values
    float integralFBx, integralFBy, integralFBz; // Integral error terms

public:
    float twoKp;    // Proportional gain
    float twoKi;    // Integral gain
    
    // Constructor, initialize parameters
    MahonyFilter(float kp = 0.400f, float ki = 0.001f);

    // Mahony filter update function
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    // Get quaternion
    void getQuaternion(float& q0_out, float& q1_out, float& q2_out, float& q3_out);
};


// Quaternion to Euler angles conversion function
void quaternionToEuler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);

#endif
