/**
 * @file imu.c
 * @author Wyatt Yu
 * @brief IMU 应用程序
 * @copyright Copyright (c) 2025
 */
#include <string.h>
#include "imu.h"

static inline double Imu_NormalizeAngle(float_t angle)
{
    return (angle > MATH_PI) ? angle - MATH_2PI : (angle < -MATH_PI) ? angle + MATH_2PI : angle;
}

static void Imu_ConvertEuler(Imu *imu)
{
    imu->raw_euler_degree.roll  = (float)imu->raw_euler.roll * 180 / MATH_PI;
    imu->raw_euler_degree.pitch = (float)imu->raw_euler.pitch * 180 / MATH_PI;
    imu->raw_euler_degree.yaw   = (float)imu->raw_euler.yaw * 180 / MATH_PI;

    imu->euler.pitch = (float)Imu_NormalizeAngle(imu->raw_euler.pitch - imu->zero_euler.pitch);
    imu->euler.roll = (float)Imu_NormalizeAngle(imu->raw_euler.roll - imu->zero_euler.roll);
    imu->euler.yaw = (float)Imu_NormalizeAngle(imu->raw_euler.yaw - imu->zero_euler.yaw);

    imu->euler_degree.roll = (float)(imu->euler.roll * 180 / MATH_PI);
    imu->euler_degree.pitch = (float)(imu->euler.pitch * 180 / MATH_PI);
    imu->euler_degree.yaw = (float)(imu->euler.yaw * 180 / MATH_PI);
}

static void Imu_ConvertQuatToEuler(Imu *imu)
{
    imu->raw_euler.roll = atan2(2 * (imu->quaternion.q0 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q3), \
            1 - 2 * (imu->quaternion.q1 * imu->quaternion.q1 + imu->quaternion.q2 * imu->quaternion.q2));
    imu->raw_euler.pitch = asin(2 * (imu->quaternion.q0 * imu->quaternion.q2 - imu->quaternion.q3 * imu->quaternion.q1));
    imu->raw_euler.yaw = atan2(2 * (imu->quaternion.q0 * imu->quaternion.q3 + imu->quaternion.q1 * imu->quaternion.q2), \
            1 - 2 * (imu->quaternion.q2 * imu->quaternion.q2 + imu->quaternion.q3 * imu->quaternion.q3));
}

void Imu_SetZero(Imu *imu)
{
    imu->zero_euler.roll = imu->raw_euler.roll;
    imu->zero_euler.pitch = imu->raw_euler.pitch;
    imu->zero_euler.yaw = imu->raw_euler.yaw;
}

void Imu_Update(Imu *imu)
{
    // read source
    imu->source.accel.x = imu->bias.accel_s.x * (imu->source.accel.x - imu->bias.accel_offset.x);
    imu->source.accel.y = imu->bias.accel_s.y * (imu->source.accel.y - imu->bias.accel_offset.y);
    imu->source.accel.z = imu->bias.accel_s.z * (imu->source.accel.z - imu->bias.accel_offset.z);
    imu->source.gyro.x  -= imu->bias.gyro.x;
    imu->source.gyro.y  -= imu->bias.gyro.y;
    imu->source.gyro.z  -= imu->bias.gyro.z;
    imu->source.magic.x -= imu->bias.magic.x;
    imu->source.magic.y -= imu->bias.magic.y;
    imu->source.magic.z -= imu->bias.magic.z;
#if 1
    if (ImuMadgwick == imu->method)
    {
        ImuMadgwick_AlgorithmUpdate(imu);
            }
    else if (ImuMahony == imu->method)
    {
        ImuMahony_AlgorithmUpdate(imu);
            }
    else if (ImuComplementaryFilter == imu->method)
    {
        ImuComplementaryFilter_AlgorithmUpdate(imu);
    }
    else
    {
        // do nothing
    }
#endif
    Imu_ConvertQuatToEuler(imu);
    Imu_ConvertEuler(imu);
}

void Imu_CalibrateGyro(Imu *imu)
{
    if (imu->calibrate_count < IMU_CALIBRATE_TIMES)
    {
        imu->bias.gyro.x  += imu->source.gyro.x;
        imu->bias.gyro.y  += imu->source.gyro.y;
        imu->bias.gyro.z  += imu->source.gyro.z;
    }
    else if (imu->calibrate_count == IMU_CALIBRATE_TIMES)
    {
        imu->bias.gyro.x  /= IMU_CALIBRATE_TIMES;
        imu->bias.gyro.y  /= IMU_CALIBRATE_TIMES;
        imu->bias.gyro.z  /= IMU_CALIBRATE_TIMES;
    }
}

void Imu_CalibrateMagic(Imu *imu)
{
    // TODO
}

// IMU 误差模型 Ameas = S * (Atrue + OFFSET)
void Imu_CalibrateAccelBias(ImuAxes *src, int32_t samples, ImuAxes *bias)
{
    ImuAxes sum = {0.0, 0.0, 0.0};
    for (int32_t i = 0; i < samples; i++)
    {
        sum.x += src[i].x;
        sum.y += src[i].y;
        sum.z += src[i].z;
    }

    bias->x = sum.x / samples;
    bias->y = sum.y / samples;
    bias->z = sum.z / samples - GRAVITY;
}

void Imu_CalibrateAccelScale(ImuAxes *src, int32_t samples, ImuAxes *scale)
{
    ImuAxes sum = {0.0, 0.0, 0.0};
    for (int32_t i = 0; i < samples; i++)
    {
        sum.x += src[i].x * src[i].x;
        sum.y += src[i].y * src[i].y;
        sum.z += src[i].z * src[i].z;
    }

    float norm = sqrt(sum.x + sum.y + sum.z);
    scale->x = GRAVITY * GRAVITY / (sqrt(sum.x / samples) * norm);
    scale->y = GRAVITY * GRAVITY / (sqrt(sum.y / samples) * norm);
    scale->z = GRAVITY * GRAVITY / (sqrt(sum.z / samples) * norm);
}

void Imu_CalibrateAccel(Imu *imu)
{
    if (imu->calibrate_count == 0)
    {
        // do nothing
    }

    if (imu->calibrate_count < IMU_CALIBRATE_TIMES) // in case of overflow
    {
        imu->bias.acc_src[imu->calibrate_count].x = imu->source.accel.x;
        imu->bias.acc_src[imu->calibrate_count].y = imu->source.accel.y;
        imu->bias.acc_src[imu->calibrate_count].z = imu->source.accel.z;
    }
    else if (imu->calibrate_count == IMU_CALIBRATE_TIMES)
    {
        Imu_CalibrateAccelBias(imu->bias.acc_src, IMU_CALIBRATE_TIMES, &imu->bias.accel_offset);
        Imu_CalibrateAccelScale(imu->bias.acc_src, IMU_CALIBRATE_TIMES, &imu->bias.accel_s);
    }
}

void Imu_InitCalibrate(Imu *imu)
{
//    memset((void *)&imu->bias, 0, sizeof(ImuCalib));
    imu->calibrate_count = 0;
    imu->state = ImuStateCalib;
    imu->bias.gyro.x = 0.0;
    imu->bias.gyro.y = 0.0;
    imu->bias.gyro.z = 0.0;
    imu->bias.magic.x = 0.0;
    imu->bias.magic.y = 0.0;
    imu->bias.magic.z = 0.0;
    imu->bias.accel_offset.x = 0.0;
    imu->bias.accel_offset.y = 0.0;
    imu->bias.accel_offset.z = 0.0;
    imu->bias.accel_s.x = 1.0;
    imu->bias.accel_s.y = 1.0;
    imu->bias.accel_s.z = 1.0;
}

void Imu_Calibrate(Imu *imu)
{
// TODO   Imu_CalibrateAccel(imu);
    Imu_CalibrateGyro(imu);
    if (imu->source.use_magic == true)
    {
        Imu_CalibrateMagic(imu);
    }
    
    imu->calibrate_count++;
    
    if (imu->calibrate_count > IMU_CALIBRATE_TIMES) // wait one tick for caculate results
    {
        imu->calibrate_count = 0;
        imu->state = ImuStateStart;
#if 0
        LOG_D("Calibrate results: \n accel_bias: %f, %f, %f\n accel_scale: %f, %f, %f\n, gyro: %f, %f, %f", \
                imu->bias.accel_offset.x, imu->bias.accel_offset.y, imu->bias.accel_offset.z, \
                imu->bias.accel_s.x, imu->bias.accel_s.y, imu->bias.accel_s.z, \
                imu->bias.gyro.x, imu->bias.gyro.y, imu->bias.gyro.z);
#endif
    }
}
