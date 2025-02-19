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
#define ADLX345_REG_DATAFORMAT  0x31
#define ADLX345_REG_DATA        0x32
#define ADLX345_REG_FIFO_CTL    0x38

typedef enum {
    Adlx345SampleRate_0_1 = 0,
    Adlx345SampleRate_0_2,
    Adlx345SampleRate_0_39,
    Adlx345SampleRate_0_78,
    Adlx345SampleRate_1_56,
    Adlx345SampleRate_3_13,
    Adlx345SampleRate_6_25,
    Adlx345SampleRate_12_5,
    Adlx345SampleRate_25,
    Adlx345SampleRate_50,
    Adlx345SampleRate_100,
    Adlx345SampleRate_200,
    Adlx345SampleRate_400,
    Adlx345SampleRate_800,
    Adlx345SampleRate_1600,
    Adlx345SampleRate_3600,
}Adlx345SampleRate;

typedef enum {
    Adlx345Range_2g = 0,            // 10-bit max
    Adlx345Range_4g,                // 11-bit max
    Adlx345Range_8g,                // 12-bit max
    Adlx345Range_16g,               // 13-bit max
}Adlx345Range;

typedef enum {
    Adlx345AddrHigh = 0x1D,        // ALT ADDRESS HIGH
    Adlx345AddrLow  = 0x53,        // ALT ADDRESS LOW
}Adlx345Addr;

typedef struct Adlx345RegBwRate_ {
    uint8_t resvd:3;
    uint8_t low_power:1;
    uint8_t rate:4;
}Adlx345RegBwRate_t;

typedef struct Adlx345RegDataFormat_ {
    uint8_t self_test:1;
    uint8_t spi:1;
    uint8_t int_invert:1;
    uint8_t resvd:1;
    uint8_t full_res:1;         //0: fix resolution mode as 10 bits; 1:full resolution mode determine by range
    uint8_t justify:1;
    uint8_t range:2;
}Adlx345RegDataFormat_t;

typedef struct Adlx345_ Adlx345_t;
typedef bool (*adlx345_func)(Adlx345_t *m, uint8_t reg, uint8_t *data, int32_t length);

struct Adlx345_ {
    Adlx345Addr addr;                   // 7-bit i2c address
    Adlx345Range range;
    Adlx345SampleRate sample_rate;
    bool fix_resolution;                 // determine by full-res & range
    adlx345_func read;
    adlx345_func write;

    uint16_t raw_data[3];
    float x;
    float y;
    float z;
    bool inited;
    float full_scale_rate;
};

void Adlx345_Init(Adlx345_t *m);
void Adlx345_Register(Adlx345_t *m, adlx345_func read, adlx345_func write);
void Adlx345_Read(Adlx345_t *m, float *x, float *y, float *z);
void Adlx345_GetSampleRate(Adlx345_t *m, Adlx345SampleRate *sample_rate);

#ifdef __cplusplus
}
#endif
#endif
