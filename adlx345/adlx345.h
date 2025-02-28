/**
 * @file adlx345.h
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#ifndef __ADLX345_H__
#define __ADLX345_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ADLX345_REG_DEVID       0X00
#define ADLX345_REG_OFFSETX     0x1E
#define ADLX345_REG_OFFSETY     0x1F
#define ADLX345_REG_OFFSETZ     0x20
#define ADLX345_REG_BW_RATE     0x2C
#define ADLX345_REG_POWER_CTL   0x2D
#define ADLX345_REG_DATAFORMAT  0x31
#define ADLX345_REG_DATA        0x32
#define ADLX345_REG_FIFO_CTL    0x38

typedef enum {
    Adlx345SampleRate_0_1  = 0,
    Adlx345SampleRate_0_2  = 1,
    Adlx345SampleRate_0_39 = 2,
    Adlx345SampleRate_0_78 = 3,
    Adlx345SampleRate_1_56 = 4,
    Adlx345SampleRate_3_13 = 5,
    Adlx345SampleRate_6_25 = 6,
    Adlx345SampleRate_12_5 = 7,
    Adlx345SampleRate_25   = 8,
    Adlx345SampleRate_50   = 9,
    Adlx345SampleRate_100  = 10,
    Adlx345SampleRate_200  = 11,
    Adlx345SampleRate_400  = 12,
    Adlx345SampleRate_800  = 13,
    Adlx345SampleRate_1600 = 14,
    Adlx345SampleRate_3600 = 15,
}Adlx345SampleRate;

typedef enum {
    Adlx345Range_2g = 0,            // 10-bit max
    Adlx345Range_4g,                // 11-bit max
    Adlx345Range_8g,                // 12-bit max
    Adlx345Range_16g,               // 13-bit max
}Adlx345Range;

typedef enum {
    Adlx345Addr_High = 0x1D,        // ALT ADDRESS HIGH
    Adlx345Addr_Low  = 0x53,        // ALT ADDRESS LOW
}Adlx345Addr;

typedef struct Adlx345RegBwRate_ {
    uint8_t resvd:3;
    uint8_t low_power:1;
    uint8_t rate:4;
}Adlx345RegBwRate;

typedef struct Adlx345RegDataFormat_ {
    uint8_t self_test:1;
    uint8_t spi:1;
    uint8_t int_invert:1;
    uint8_t resvd:1;
    uint8_t full_res:1;         //0: fix resolution mode as 10 bits; 1:full resolution mode determine by range
    uint8_t justify:1;
    uint8_t range:2;
}Adlx345RegDataFormat;

typedef struct Adlx345Axes_ {
    float x;
    float y;
    float z;
}Adlx345Axes;

typedef bool (*Adlx345_I2cMemFunc)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t length);

typedef struct Adlx345_ {
    Adlx345Addr addr;                   // 7-bit i2c address
    Adlx345Range range;
    Adlx345SampleRate sample_rate;
    Adlx345_I2cMemFunc read;
    Adlx345_I2cMemFunc write;

    bool fix_resolution;                 // determine by full-res & range
    int16_t raw_data[3];
    Adlx345Axes axes;
    bool inited;
    float full_scale_rate;
}Adlx345;

void Adlx345_Init(Adlx345 *m);
void Adlx345_Register(Adlx345 *m, Adlx345_I2cMemFunc read, Adlx345_I2cMemFunc write);
bool Adlx345_Read(Adlx345 *m, Adlx345Axes *axes);
void Adlx345_GetSampleRate(Adlx345 *m, Adlx345SampleRate *sample_rate);

#ifdef __cplusplus
}
#endif
#endif
