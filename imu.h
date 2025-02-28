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

#define GRAVITY                 (9.81)
#define DEGREE2RAD(x)           ((x) * MATH_PI / 180.0)
#define RAD2DEGREE(x)           ((x) * 180.0 / MATH_PI)
#define IMU_CALIBRATE_TIMES     500

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

typedef struct ImuAxes_ {
    volatile float x;
    volatile float y;
    volatile float z;
}ImuAxes;

typedef struct ImuSource_ {
    ImuAxes accel;         // m/s2
    ImuAxes gyro;          // rad/s
    ImuAxes magic;         // Gauss
    float accel_temperature;
    float gyro_temperature;
    float magic_temperature;
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

// Ameas = S * (Atrue + OFFSET), 静止条件下最小二乘法校准参数
typedef struct ImuCalib_ {
    ImuAxes acc_src[IMU_CALIBRATE_TIMES];
    ImuAxes accel_s;
    ImuAxes accel_offset;
    ImuAxes gyro;          // rad/s
    ImuAxes magic;         // Gauss
}ImuCalib;

typedef struct Imu_ Imu;
struct Imu_ {
    volatile ImuState state;
    ImuMethod method;           // 滤波方法
    ImuSource source;           // 源数据
    ImuCalib bias;          //初始值校准
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