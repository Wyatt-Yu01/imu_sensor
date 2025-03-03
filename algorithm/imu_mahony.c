/**
 * @file imu_mahony.c
 * @author Wyatt Yu
 * @brief IMU algorithm based on the Mahony algorithm
 * @copyright Copyright (c) 2025
 */

#include "app_common.h"
#include "imu.h"

void ImuMahony_AlgorithmUpdate(Imu *imu)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

    if (imu->source.use_magic)
    {
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
        float hx, hy, bx, bz;
        float halfwx, halfwy, halfwz;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((imu->source.accel.x == 0.0f) && (imu->source.accel.y == 0.0f) && (imu->source.accel.z == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = InvSqrt(imu->source.accel.x * imu->source.accel.x + imu->source.accel.y * imu->source.accel.y + imu->source.accel.z * imu->source.accel.z);
            imu->source.accel.x *= recipNorm;
            imu->source.accel.y *= recipNorm;
            imu->source.accel.z *= recipNorm;     

            // Normalise magnetometer measurement
            recipNorm = InvSqrt(imu->source.magic.x * imu->source.magic.x + imu->source.magic.y * imu->source.magic.y + imu->source.magic.z * imu->source.magic.z);
            imu->source.magic.x *= recipNorm;
            imu->source.magic.y *= recipNorm;
            imu->source.magic.z *= recipNorm;   

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = imu->quaternion.q0 * imu->quaternion.q0;
            q0q1 = imu->quaternion.q0 * imu->quaternion.q1;
            q0q2 = imu->quaternion.q0 * imu->quaternion.q2;
            q0q3 = imu->quaternion.q0 * imu->quaternion.q3;
            q1q1 = imu->quaternion.q1 * imu->quaternion.q1;
            q1q2 = imu->quaternion.q1 * imu->quaternion.q2;
            q1q3 = imu->quaternion.q1 * imu->quaternion.q3;
            q2q2 = imu->quaternion.q2 * imu->quaternion.q2;
            q2q3 = imu->quaternion.q2 * imu->quaternion.q3;
            q3q3 = imu->quaternion.q3 * imu->quaternion.q3;   

            // Reference direction of Earth's magnetic field
            hx = 2.0f * (imu->source.magic.x * (0.5f - q2q2 - q3q3) + imu->source.magic.y * (q1q2 - q0q3) + imu->source.magic.z * (q1q3 + q0q2));
            hy = 2.0f * (imu->source.magic.x * (q1q2 + q0q3) + imu->source.magic.y * (0.5f - q1q1 - q3q3) + imu->source.magic.z * (q2q3 - q0q1));
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0f * (imu->source.magic.x * (q1q3 - q0q2) + imu->source.magic.y * (q2q3 + q0q1) + imu->source.magic.z * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  

            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (imu->source.accel.y * halfvz - imu->source.accel.z * halfvy) + (imu->source.magic.y * halfwz - imu->source.magic.z * halfwy);
            halfey = (imu->source.accel.z * halfvx - imu->source.accel.x * halfvz) + (imu->source.magic.z * halfwx - imu->source.magic.x * halfwz);
            halfez = (imu->source.accel.x * halfvy - imu->source.accel.y * halfvx) + (imu->source.magic.x * halfwy - imu->source.magic.y * halfwx);

            // Compute and apply integral feedback if enabled
            if(imu->ki_gain > 0.0f) {
                integralFBx        += imu->ki_gain * halfex * (1.0f / imu->samp_freq);  // integral error scaled by Ki
                integralFBy        += imu->ki_gain * halfey * (1.0f / imu->samp_freq);
                integralFBz        += imu->ki_gain * halfez * (1.0f / imu->samp_freq);
                imu->source.gyro.x += integralFBx;                                      // apply integral feedback
                imu->source.gyro.y += integralFBy;
                imu->source.gyro.z += integralFBz;
            }
            else {
                integralFBx = 0.0f;	// prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            imu->source.gyro.x += imu->kp_gain * halfex;
            imu->source.gyro.y += imu->kp_gain * halfey;
            imu->source.gyro.z += imu->kp_gain * halfez;
        }

        // Integrate rate of change of quaternion
        imu->source.gyro.x *= (0.5f * (1.0f / imu->samp_freq));                                                                // pre-multiply common factors
        imu->source.gyro.y *= (0.5f * (1.0f / imu->samp_freq));
        imu->source.gyro.z *= (0.5f * (1.0f / imu->samp_freq));
        qa                  = imu->quaternion.q0;
        qb                  = imu->quaternion.q1;
        qc                  = imu->quaternion.q2;
        imu->quaternion.q0 += (-qb * imu->source.gyro.x - qc * imu->source.gyro.y - imu->quaternion.q3 * imu->source.gyro.z);
        imu->quaternion.q1 += (qa * imu->source.gyro.x + qc * imu->source.gyro.z - imu->quaternion.q3 * imu->source.gyro.y);
        imu->quaternion.q2 += (qa * imu->source.gyro.y - qb * imu->source.gyro.z + imu->quaternion.q3 * imu->source.gyro.x);
        imu->quaternion.q3 += (qa * imu->source.gyro.z + qb * imu->source.gyro.y - qc * imu->source.gyro.x);

          // Normalise quaternion
        recipNorm           = InvSqrt(imu->quaternion.q0 * imu->quaternion.q0 + imu->quaternion.q1 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q2 + imu->quaternion.q3 * imu->quaternion.q3);
        imu->quaternion.q0 *= recipNorm;
        imu->quaternion.q1 *= recipNorm;
        imu->quaternion.q2 *= recipNorm;
        imu->quaternion.q3 *= recipNorm;
    }
    else 
    {
        if(!((imu->source.accel.x == 0.0f) && (imu->source.accel.y == 0.0f) && (imu->source.accel.z == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm            = InvSqrt(imu->source.accel.x * imu->source.accel.x + imu->source.accel.y * imu->source.accel.y + imu->source.accel.z * imu->source.accel.z);
            imu->source.accel.x *= recipNorm;
            imu->source.accel.y *= recipNorm;
            imu->source.accel.z *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = imu->quaternion.q1 * imu->quaternion.q3 - imu->quaternion.q0 * imu->quaternion.q2;
            halfvy = imu->quaternion.q0 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q3;
            halfvz = imu->quaternion.q0 * imu->quaternion.q0 - 0.5f + imu->quaternion.q3 * imu->quaternion.q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (imu->source.accel.y * halfvz - imu->source.accel.z * halfvy);
            halfey = (imu->source.accel.z * halfvx - imu->source.accel.x * halfvz);
            halfez = (imu->source.accel.x * halfvy - imu->source.accel.y * halfvx);

            // Compute and apply integral feedback if enabled
            if(imu->ki_gain > 0.0f) {
                integralFBx        += imu->ki_gain * halfex * (1.0f / imu->samp_freq);  // integral error scaled by Ki
                integralFBy        += imu->ki_gain * halfey * (1.0f / imu->samp_freq);
                integralFBz        += imu->ki_gain * halfez * (1.0f / imu->samp_freq);
                imu->source.gyro.x += integralFBx;                                      // apply integral feedback
                imu->source.gyro.y += integralFBy;
                imu->source.gyro.z += integralFBz;
            }
            else {
                integralFBx = 0.0f;	// prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            imu->source.gyro.x += imu->kp_gain * halfex;
            imu->source.gyro.y += imu->kp_gain * halfey;
            imu->source.gyro.z += imu->kp_gain * halfez;
        }

                                                                 // Integrate rate of change of quaternion
        imu->source.gyro.x *= (0.5f * (1.0f / imu->samp_freq));  // pre-multiply common factors
        imu->source.gyro.y *= (0.5f * (1.0f / imu->samp_freq));
        imu->source.gyro.z *= (0.5f * (1.0f / imu->samp_freq));
        qa                  = imu->quaternion.q0;
        qb                  = imu->quaternion.q1;
        qc                  = imu->quaternion.q2;
        imu->quaternion.q0 += (-qb * imu->source.gyro.x - qc * imu->source.gyro.y - imu->quaternion.q3 * imu->source.gyro.z);
        imu->quaternion.q1 += (qa * imu->source.gyro.x + qc * imu->source.gyro.z - imu->quaternion.q3 * imu->source.gyro.y);
        imu->quaternion.q2 += (qa * imu->source.gyro.y - qb * imu->source.gyro.z + imu->quaternion.q3 * imu->source.gyro.x);
        imu->quaternion.q3 += (qa * imu->source.gyro.z + qb * imu->source.gyro.y - qc * imu->source.gyro.x);

        // Normalise quaternion
        recipNorm = InvSqrt(imu->quaternion.q0 * imu->quaternion.q0 + imu->quaternion.q1 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q2 + imu->quaternion.q3 * imu->quaternion.q3);
        imu->quaternion.q0 *= recipNorm;
        imu->quaternion.q1 *= recipNorm;
        imu->quaternion.q2 *= recipNorm;
        imu->quaternion.q3 *= recipNorm;
    }
}


