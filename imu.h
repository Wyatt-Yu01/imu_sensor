/**
 * @file imu.h
 * @author Wyatt Yu
 * @brief IMU 应用程序 头文件
 * @copyright Copyright (c) 2025
 */

#ifndef __IMU_H__
#define __IMU_H__
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "rtdevice.h"
#include "app_common.h"

typedef enum {
    ImuMadgwick = 1,
    ImuMahony   = 2,
    ImuComplementaryFilter = 3,
}ImuMethod;

typedef enum {
    ImuStateStart      = 1,
    ImuStateRuning     = 2,
    ImuStateStartCalib = 3,
    ImuStateCalib      = 4
}ImuState;

typedef enum {
    ImuSetGyroRange  = 0,
    ImuSetAccelRange = 1,
    ImuSetDlpf       = 2,
    ImuSetSampleRate = 3,
    ImuSetSleep      = 4
}ImuConfigCmd;

typedef enum {
    ImuGyroRange_250DPS  = 250,
    ImuGyroRange_500DPS  = 500,
    ImuGyroRange_1000DPS = 1000,
    ImuGyroRange_2000DPS = 2000
} ImuGyroRange;

typedef enum {
    ImuAccelRange_2G  = 2,
    ImuAccelRange_4G  = 4,
    ImuAccelRange_8G  = 8,
    ImuAccelRange_16G = 18
} ImuAccelRange;

typedef enum {
    ImuDlpf_Disable = 0,
    ImuDlpf_188HZ   = 1,
    ImuDlpf_98HZ    = 2,
    ImuDlpf_42HZ    = 3,
    ImuDlpf_20HZ    = 4,
    ImuDlpf_10HZ    = 5,
    ImuDlpf_5HZ     = 6,
} ImuDlpf;

typedef struct Imu3Axes_ {
    volatile float x;
    volatile float y;
    volatile float z;
}Imu3Axes;

typedef struct ImuConfig_ {
    ImuAccelRange accel_range;
    ImuGyroRange gyro_range;
    ImuDlpf dlpf;
    uint32_t resolution;
}ImuConfig;

typedef struct ImuSource_ {
    Imu3Axes accel;
    Imu3Axes gyro;
    Imu3Axes magic;
    bool use_magic;
}ImuSource;

typedef struct ImuEuler_ {
    float roll;
    float pitch;
    float yaw;
}ImuEuler;

typedef struct ImuQuaternion_ {
    volatile float q0;
    volatile float q1;
    volatile float q2;
    volatile float q3;
}ImuQuaternion;

typedef struct Imu_ Imu;
    
struct Imu_ {
    volatile ImuState state;
    ImuMethod method;           // 滤波方法
    ImuSource source;           // 源数据
    ImuSource src_bias;          // 初始值校准
    ImuConfig imu_config;       // 芯片配置项
    ImuQuaternion quaternion;   // 四元数
    ImuEuler raw_euler;         // 欧拉角 rad
    ImuEuler raw_euler_degree;  // 欧拉角 degree
    ImuEuler zero_euler;        // 用户定义的零点位置, rad
    ImuEuler euler;             // 相对零点位置的角度, rad
    ImuEuler euler_degree;      // 相对零点位置的角度, degree
    int32_t samp_freq;          // 采样频率
    float kp_gain;              // 比例增益 Kp
    float ki_gain;              // Ki for mahony, beta for madgwick
    float comple_filter_alpha;  // 互补滤波算法系数， 即陀螺仪权重
    void (*read_source)(Imu *imu);
    volatile int32_t calibrate_count;
};

void ImuMadgwick_AlgorithmUpdate(Imu *imu);
void ImuMahony_AlgorithmUpdate(Imu *imu);
void Imu_SetZero(Imu *imu);
void Imu_Update(Imu *imu);
void Imu_InitCalibrate(Imu *imu);
void Imu_Calibrate(Imu *imu);
void ImuComplementaryFilter_AlgorithmUpdate(Imu *imu);
#endif