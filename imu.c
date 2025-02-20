/**
 * @file imu.c
 * @author Wyatt Yu
 * @brief IMU 应用程序
 * @copyright Copyright (c) 2025
 */
#include "imu.h"

#define DBG_SECTION_NAME "imu"
#define DBG_COLOR
#define DBG_LEVEL DBG_INFO
#include <rtdbg.h>

#define IMU_CALIBRATE_TIMES     1000

static inline float_t Imu_NormalizeAngle(float_t angle)
{
    return (angle > MATH_PI) ? angle - MATH_2PI : (angle < -MATH_PI) ? angle + MATH_2PI : angle;
}

static void Imu_ConvertEuler(Imu *imu)
{
    imu->raw_euler_degree.roll  = imu->raw_euler.roll * 180 / MATH_PI;
    imu->raw_euler_degree.pitch = imu->raw_euler.pitch * 180 / MATH_PI;
    imu->raw_euler_degree.yaw   = imu->raw_euler.yaw * 180 / MATH_PI;

    imu->euler.pitch = Imu_NormalizeAngle(imu->raw_euler.pitch - imu->zero_euler.pitch);
    imu->euler.roll = Imu_NormalizeAngle(imu->raw_euler.roll - imu->zero_euler.roll);
    imu->euler.yaw = Imu_NormalizeAngle(imu->raw_euler.yaw - imu->zero_euler.yaw);

    imu->euler_degree.roll = imu->euler.roll * 180 / MATH_PI;
    imu->euler_degree.pitch = imu->euler.pitch * 180 / MATH_PI;
    imu->euler_degree.yaw = imu->euler.yaw * 180 / MATH_PI;
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
    imu->source.accel.x -= imu->src_bias.accel.x;
    imu->source.accel.y -= imu->src_bias.accel.y;
    imu->source.accel.z -= imu->src_bias.accel.z;
    imu->source.gyro.x  -= imu->src_bias.gyro.x;
    imu->source.gyro.y  -= imu->src_bias.gyro.y;
    imu->source.gyro.z  -= imu->src_bias.gyro.z;
    imu->source.magic.x -= imu->src_bias.magic.x;
    imu->source.magic.y -= imu->src_bias.magic.y;
    imu->source.magic.z -= imu->src_bias.magic.z;

    if (ImuMadgwick == imu->method)
    {
        ImuMadgwick_AlgorithmUpdate(imu);
    }
    else if (ImuMahony == imu->method)
    {
        ImuMahony_AlgorithmUpdate(imu);
    }
    else
    {
        //
    }
    Imu_ConvertQuatToEuler(imu);
    Imu_ConvertEuler(imu);
}

void Imu_InitCalibrate(Imu *imu)
{
    memset(imu->src_bias, 0, sizeof(ImuSource));
    imu->calibrate_count = 0;
    imu->src_bias.use_magic = imu->source.use_magic;
    imu->state = ImuStateCalib;
}

void Imu_Calibrate(Imu *imu)
{
    imu->src_bias.accel.x += imu->source.accel.x;
    imu->src_bias.accel.y += imu->source.accel.y;
    imu->src_bias.accel.z += imu->source.accel.z;
    imu->src_bias.gyro.x  += imu->source.gyro.x;
    imu->src_bias.gyro.y  += imu->source.gyro.y;
    imu->src_bias.gyro.z  += imu->source.gyro.z;
    imu->src_bias.magic.x += imu->source.magic.x;
    imu->src_bias.magic.y += imu->source.magic.y;
    imu->src_bias.magic.z += imu->source.magic.z;
    imu->calibrate_count++;
    if (imu->calibrate_count >= IMU_CALIBRATE_TIMES)
    {
        imu->src_bias.accel.x /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.accel.y /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.accel.z /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.gyro.x  /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.gyro.y  /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.gyro.z  /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.magic.x /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.magic.y /= (IMU_CALIBRATE_TIMES * 1.0);
        imu->src_bias.magic.z /= (IMU_CALIBRATE_TIMES * 1.0);
        rt_kprintf("offset: %f %f %f %f %f %f %f %f %f\n", imu->src_bias.accel.x, \
                imu->src_bias.accel.y, imu->src_bias.accel.z, \
                imu->src_bias.gyro.x, imu->src_bias.gyro.y, \
                imu->src_bias.gyro.z, imu->src_bias.magic.x, \
                imu->src_bias.magic.y, imu->src_bias.magic.z);
        imu->state = ImuStateRuning;
    }
    
    
}
