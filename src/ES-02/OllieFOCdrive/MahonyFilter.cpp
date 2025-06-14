#include "MahonyFilter.h"
#include <math.h>

// Constructor implementation
MahonyFilter::MahonyFilter(float kp, float ki) : twoKp(kp), twoKi(ki), 
    q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f), 
    integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f) {}

// Mahony filter update function implementation
void MahonyFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // If accelerometer measurement is zero, skip filter update
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimate gravity direction
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Calculate error between measurement and estimate
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Integrate error
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Proportional error
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Quaternion differential equation, using dt
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    float qa = q0;
    float qb = q1;
    float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// Get quaternion function implementation
void MahonyFilter::getQuaternion(float& q0_out, float& q1_out, float& q2_out, float& q3_out) {
    q0_out = q0;
    q1_out = q1;
    q2_out = q2;
    q3_out = q3;
}

// Quaternion to Euler angles conversion function
void quaternionToEuler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw) {
    *roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    *pitch = asin(2 * (q0 * q2 - q3 * q1));
    *yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));

    // Convert radians to degrees
    *roll *= (180.0f / 3.1415926f);
    *pitch *= (180.0f / 3.1415926f);
    *yaw *= (180.0f / 3.1415926f);
}
