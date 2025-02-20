/**
 * @file imu_madgwick.c
 * @author Wyatt Yu
 * @brief IMU algorithm based on the madgwick algorithm
 * @copyright Copyright (c) 2025
 */
#include "app_common.h"
#include "imu.h"

void ImuMadgwick_AlgorithmUpdate(Imu *imu)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;

    if (imu->source.use_magic) 
    {
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx;
        float _2bx, _2bz;
        float _4bx, _4bz;
        float _2q0q2, _2q2q3;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (imu->quaternion.q1 * imu->source.gyro.x - imu->quaternion.q2 * imu->source.gyro.y - imu->quaternion.q3 * imu->source.gyro.z);
        qDot2 = 0.5f * (imu->quaternion.q0 * imu->source.gyro.x + imu->quaternion.q2 * imu->source.gyro.z - imu->quaternion.q3 * imu->source.gyro.y);
        qDot3 = 0.5f * (imu->quaternion.q0 * imu->source.gyro.y - imu->quaternion.q1 * imu->source.gyro.z + imu->quaternion.q3 * imu->source.gyro.x);
        qDot4 = 0.5f * (imu->quaternion.q0 * imu->source.gyro.z + imu->quaternion.q1 * imu->source.gyro.y - imu->quaternion.q2 * imu->source.gyro.x);

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
            _2q0mx = 2.0f * imu->quaternion.q0 * imu->source.magic.x;
            _2q0my = 2.0f * imu->quaternion.q0 * imu->source.magic.y;
            _2q0mz = 2.0f * imu->quaternion.q0 * imu->source.magic.z;
            _2q1mx = 2.0f * imu->quaternion.q1 * imu->source.magic.x;
            _2q0 = 2.0f * imu->quaternion.q0;
            _2q1 = 2.0f * imu->quaternion.q1;
            _2q2 = 2.0f * imu->quaternion.q2;
            _2q3 = 2.0f * imu->quaternion.q3;
            _2q0q2 = 2.0f * imu->quaternion.q0 * imu->quaternion.q2;
            _2q2q3 = 2.0f * imu->quaternion.q2 * imu->quaternion.q3;
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
            hx = imu->source.magic.x * q0q0 - _2q0my * imu->quaternion.q3 + _2q0mz * imu->quaternion.q2 + imu->source.magic.x * q1q1 + _2q1 * imu->source.magic.y * imu->quaternion.q2 + _2q1 * imu->source.magic.z * imu->quaternion.q3 - imu->source.magic.x * q2q2 - imu->source.magic.x * q3q3;
            hy = _2q0mx * imu->quaternion.q3 + imu->source.magic.y * q0q0 - _2q0mz * imu->quaternion.q1 + _2q1mx * imu->quaternion.q2 - imu->source.magic.y * q1q1 + imu->source.magic.y * q2q2 + _2q2 * imu->source.magic.z * imu->quaternion.q3 - imu->source.magic.y * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * imu->quaternion.q2 + _2q0my * imu->quaternion.q1 + imu->source.magic.z * q0q0 + _2q1mx * imu->quaternion.q3 - imu->source.magic.z * q1q1 + _2q2 * imu->source.magic.y * imu->quaternion.q3 - imu->source.magic.z * q2q2 + imu->source.magic.z * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - imu->source.accel.x) + _2q1 * (2.0f * q0q1 + _2q2q3 - imu->source.accel.y) - _2bz * imu->quaternion.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu->source.magic.x) + (-_2bx * imu->quaternion.q3 + _2bz * imu->quaternion.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu->source.magic.y) + _2bx * imu->quaternion.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu->source.magic.z);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - imu->source.accel.x) + _2q0 * (2.0f * q0q1 + _2q2q3 - imu->source.accel.y) - 4.0f * imu->quaternion.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - imu->source.accel.z) + _2bz * imu->quaternion.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu->source.magic.x) + (_2bx * imu->quaternion.q2 + _2bz * imu->quaternion.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu->source.magic.y) + (_2bx * imu->quaternion.q3 - _4bz * imu->quaternion.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu->source.magic.z);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - imu->source.accel.x) + _2q3 * (2.0f * q0q1 + _2q2q3 - imu->source.accel.y) - 4.0f * imu->quaternion.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - imu->source.accel.z) + (-_4bx * imu->quaternion.q2 - _2bz * imu->quaternion.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu->source.magic.x) + (_2bx * imu->quaternion.q1 + _2bz * imu->quaternion.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu->source.magic.y) + (_2bx * imu->quaternion.q0 - _4bz * imu->quaternion.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu->source.magic.z);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - imu->source.accel.x) + _2q2 * (2.0f * q0q1 + _2q2q3 - imu->source.accel.y) + (-_4bx * imu->quaternion.q3 + _2bz * imu->quaternion.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - imu->source.magic.x) + (-_2bx * imu->quaternion.q0 + _2bz * imu->quaternion.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - imu->source.magic.y) + _2bx * imu->quaternion.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - imu->source.magic.z);
            recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= imu->ki_gain * s0;
            qDot2 -= imu->ki_gain * s1;
            qDot3 -= imu->ki_gain * s2;
            qDot4 -= imu->ki_gain * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        imu->quaternion.q0 += qDot1 * (1.0f / imu->samp_freq);
        imu->quaternion.q1 += qDot2 * (1.0f / imu->samp_freq);
        imu->quaternion.q2 += qDot3 * (1.0f / imu->samp_freq);
        imu->quaternion.q3 += qDot4 * (1.0f / imu->samp_freq);

        // Normalise quaternion
        recipNorm = InvSqrt(imu->quaternion.q0 * imu->quaternion.q0 + imu->quaternion.q1 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q2 + imu->quaternion.q3 * imu->quaternion.q3);
        imu->quaternion.q0 *= recipNorm;
        imu->quaternion.q1 *= recipNorm;
        imu->quaternion.q2 *= recipNorm;
        imu->quaternion.q3 *= recipNorm;
    }
    else
    {
        float _4q0, _4q1, _4q2;
        float _8q1, _8q2;
        float q0q0, q1q1, q2q2, q3q3;
        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (imu->quaternion.q1 * imu->source.gyro.x - imu->quaternion.q2 * imu->source.gyro.y - imu->quaternion.q3 * imu->source.gyro.z);
        qDot2 = 0.5f * (imu->quaternion.q0 * imu->source.gyro.x + imu->quaternion.q2 * imu->source.gyro.z - imu->quaternion.q3 * imu->source.gyro.y);
        qDot3 = 0.5f * (imu->quaternion.q0 * imu->source.gyro.y - imu->quaternion.q1 * imu->source.gyro.z + imu->quaternion.q3 * imu->source.gyro.x);
        qDot4 = 0.5f * (imu->quaternion.q0 * imu->source.gyro.z + imu->quaternion.q1 * imu->source.gyro.y - imu->quaternion.q2 * imu->source.gyro.x);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((imu->source.accel.x == 0.0f) && (imu->source.accel.y == 0.0f) && (imu->source.accel.z == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = InvSqrt(imu->source.accel.x * imu->source.accel.x + imu->source.accel.y * imu->source.accel.y + imu->source.accel.z * imu->source.accel.z);
            imu->source.accel.x *= recipNorm;
            imu->source.accel.y *= recipNorm;
            imu->source.accel.z *= recipNorm;   

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * imu->quaternion.q0;
            _2q1 = 2.0f * imu->quaternion.q1;
            _2q2 = 2.0f * imu->quaternion.q2;
            _2q3 = 2.0f * imu->quaternion.q3;
            _4q0 = 4.0f * imu->quaternion.q0;
            _4q1 = 4.0f * imu->quaternion.q1;
            _4q2 = 4.0f * imu->quaternion.q2;
            _8q1 = 8.0f * imu->quaternion.q1;
            _8q2 = 8.0f * imu->quaternion.q2;
            q0q0 = imu->quaternion.q0 * imu->quaternion.q0;
            q1q1 = imu->quaternion.q1 * imu->quaternion.q1;
            q2q2 = imu->quaternion.q2 * imu->quaternion.q2;
            q3q3 = imu->quaternion.q3 * imu->quaternion.q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * imu->source.accel.x + _4q0 * q1q1 - _2q1 * imu->source.accel.y;
            s1 = _4q1 * q3q3 - _2q3 * imu->source.accel.x + 4.0f * q0q0 * imu->quaternion.q1 - _2q0 * imu->source.accel.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * imu->source.accel.z;
            s2 = 4.0f * q0q0 * imu->quaternion.q2 + _2q0 * imu->source.accel.x + _4q2 * q3q3 - _2q3 * imu->source.accel.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * imu->source.accel.z;
            s3 = 4.0f * q1q1 * imu->quaternion.q3 - _2q1 * imu->source.accel.x + 4.0f * q2q2 * imu->quaternion.q3 - _2q2 * imu->source.accel.y;
            recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= imu->ki_gain * s0;
            qDot2 -= imu->ki_gain * s1;
            qDot3 -= imu->ki_gain * s2;
            qDot4 -= imu->ki_gain * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        imu->quaternion.q0 += qDot1 * (1.0f / imu->samp_freq);
        imu->quaternion.q1 += qDot2 * (1.0f / imu->samp_freq);
        imu->quaternion.q2 += qDot3 * (1.0f / imu->samp_freq);
        imu->quaternion.q3 += qDot4 * (1.0f / imu->samp_freq);

        // Normalise quaternion
        recipNorm = InvSqrt(imu->quaternion.q0 * imu->quaternion.q0 + imu->quaternion.q1 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q2 + imu->quaternion.q3 * imu->quaternion.q3);
        imu->quaternion.q0 *= recipNorm;
        imu->quaternion.q1 *= recipNorm;
        imu->quaternion.q2 *= recipNorm;
        imu->quaternion.q3 *= recipNorm;
    }
}