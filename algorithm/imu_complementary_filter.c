/**
 * @file imu_complementaryf_filter.c
 * @author Wyatt Yu
 * @brief IMU 互补滤波算法的实现, 互补滤波算法必须包含磁力计数据
 * @copyright Copyright (c) 2025
 */
#include "app_common.h"
#include "imu.h"

void ImuComplementaryFilter_AlgorithmUpdate(Imu *imu)
{
    float gyro_pitch = imu->source.gyro.x * imu->comple_filter_alpha;
    float gyro_roll = imu->source.gyro.y * imu->comple_filter_alpha;
    float gyro_yaw = imu->source.gyro.z * imu->comple_filter_alpha; 

    float accel_pitch = atan2f(imu->source.accel.y, imu->source.accel.z);
    float accel_roll = atan2f(-imu->source.accel.x, \
                sqrtf(imu->source.accel.y * imu->source.accel.y + \
                imu->source.accel.z * imu->source.accel.z));
    imu->raw_euler.pitch = imu->comple_filter_alpha * (imu->raw_euler.pitch + gyro_pitch) + \
                    (1 - imu->comple_filter_alpha) * accel_roll;
    imu->raw_euler.roll = imu->comple_filter_alpha * (imu->raw_euler.roll + gyro_roll) + \
                    (1 - imu->comple_filter_alpha) * accel_pitch;
    if (imu->source.use_magic)
    {
        float mag_x = imu->source.magic.x * cosf(imu->raw_euler.pitch) + imu->source.magic.z * sinf(imu->raw_euler.pitch);
        float mag_y = imu->source.magic.x * sinf(imu->raw_euler.roll) * sinf(imu->raw_euler.pitch) +
                        imu->source.magic.y * cosf(imu->raw_euler.roll) -
                        imu->source.magic.z * sinf(imu->raw_euler.roll) * cosf(imu->raw_euler.pitch);
        float accel_yaw = atan2f(-mag_y, mag_x);  // 基于磁力计的偏航角

        imu->raw_euler.yaw = imu->comple_filter_alpha * (imu->raw_euler.yaw + gyro_yaw) + \
                        (1 - imu->comple_filter_alpha) * accel_yaw;
    }
    else
    {
        imu->raw_euler.yaw = 0;
    }
}