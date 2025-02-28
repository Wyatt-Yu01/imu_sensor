/**
 * @file itg3205.h
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#ifndef __ITG3205_H__
#define __ITG3205_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ITG3205_REG_DEVID                   0
#define ITG3205_REG_SAMPLE_RATE_DIV         21
#define ITG3205_REG_DLPF                    22
#define ITG3205_REG_INT_CFG                 23
#define ITG3205_REG_INT_STATUS              23
#define ITG3205_REG_DATA                    27
#define ITG3205_REG_PWR                     62

typedef enum {
    Itg3205Addr_Low  = 0x68,                 // AD0 = LOW
    Itg3205Addr_High = 0x69,                 // AD0 = HIGH
}Itg3205Addr;

typedef enum {
    Itg3205DlpfBaudrate_256 = 0,   // internal sample rate = 8KHZ, other sample rate = 1KHZ
    Itg3205DlpfBaudrate_188 = 1,
    Itg3205DlpfBaudrate_98  = 2,
    Itg3205DlpfBaudrate_42  = 3,
    Itg3205DlpfBaudrate_20  = 4,
    Itg3205DlpfBaudrate_10  = 5,
    Itg3205DlpfBaudrate_5   = 6,
}Itg3205DlpfBaudrate;

typedef enum {
    Itg3205ClockSource_Internal = 0,
    Itg3205ClockSource_PllX,
    Itg3205ClockSource_PllY,
    Itg3205ClockSource_PllZ,
    Itg3205ClockSource_EXT32K,
    Itg3205ClockSource_EXT19M,
} Itg3205ClockSource;

typedef struct Itg3205Axis_ {
    float x;
    float y;
    float z;
}Itg3205Axes;  // 修正拼写错误

typedef struct Itg3205RegDlpf_ {
    uint8_t resvd:5;
    Itg3205DlpfBaudrate dlpf:3;
} Itg3205RegDlpf;

typedef struct Itg3205RegPower_ {
    uint8_t soft_reset  : 1;
    uint8_t sleep       : 1;
    uint8_t stby_x      : 1;
    uint8_t stby_y      : 1;
    uint8_t stby_z      : 1;
    uint8_t clock_source: 3;
}Itg3205RegPower;

typedef bool (*Itg3205_I2cMemFunc)(uint8_t addr, uint8_t reg, uint8_t *data, int32_t length);

typedef struct Itg3205_ {
    Itg3205Addr addr;                 // 7-bit i2c address
    // sample rate = Finternal / (1 + divider), Finternal = 8KHz if dlpf = 0; Finternal = 1KHz if dlpf != 0
    uint8_t sample_div;                
    Itg3205DlpfBaudrate lpf;
    Itg3205_I2cMemFunc read;
    Itg3205_I2cMemFunc write;

    int32_t sample_rate;
    int16_t raw_data[4];
    float temperature;
    Itg3205Axes axes;
    bool inited;
} Itg3205;

void Itg3205_Register(Itg3205 *m, Itg3205_I2cMemFunc read, Itg3205_I2cMemFunc write);
void Itg3205_Init(Itg3205 *m);
bool Itg3205_Read(Itg3205 *m, float *temp, Itg3205Axes *axes);  // 修正拼写错误
int32_t Itg3205_GetSampleRate(Itg3205 *m);

#ifdef __cplusplus
}
#endif
#endif