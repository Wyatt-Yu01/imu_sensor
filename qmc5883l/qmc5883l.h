/**
 * @file qmc5883l.h
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    Qmc5883lMode_Standby    = 0,
    Qmc5883lMode_Continuous = 1,
    Qmc5883lMode_reserve    = 2
}Qmc5883lMode;

typedef enum {
    Qmc5883lRate_10hz  = 0,
    Qmc5883lRate_50hz  = 1,
    Qmc5883lRate_100hz = 2,
    Qmc5883lRate_200hz = 3,
}Qmc5883lRate;

typedef enum {
    Qmc5883lRange_2gauss  = 0,
    Qmc5883lRange_8gauss  = 1,
    Qmc5883lRange_reserve = 2
}Qmc5883lRange;

typedef enum {
    Qmc5883lOsr_512 = 0,
    Qmc5883lOsr_256 = 1,
    Qmc5883lOsr_128 = 2,
    Qmc5883lOsr_64  = 3
}Qmc5883lOverSampleRatio;

typedef enum {
    Qmc5883lCmd_Mode            = 1,
    Qmc5883lCmd_DataRate        = 2,
    Qmc5883lCmd_FullScall       = 3,
    Qmc5883lCmd_OverSampleRate  = 4,
    Qmc5883lCmd_Reset           = 5,
    Qmc5883lCmd_ResetPeriod     = 6,
    Qmc5883lCmd_RolPnt          = 7,   // pointer roll over function
    Qmc5883lCmd_InterruptEnable = 8,
    Qmc5883lCmd_ChipId          = 9,
    Qmc5883lCmd_ReadStatus      = 10,
}Qmc5883lCmd;

typedef struct Qmc5883lReg_ {
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t z_lsb;
    uint8_t z_msb;
    struct status {
        uint8_t resvd:5;
        uint8_t dor:1;          // 1: data skipped for reading
        uint8_t ovl:1;          // overflow flag. 1: overflow
        uint8_t drdy:1;         // data ready
    } status;
    uint8_t temp_lsb;
    uint8_t temp_msb;
    struct control {                        // read/write
        Qmc5883lOverSampleRatio osr:2;
        Qmc5883lRange rng:2;
        Qmc5883lRate odr:2;
        Qmc5883lMode mode:2;
    } control;
    struct control2 {                       // read/write
        uint8_t soft_reset:1;
        uint8_t rol_pnt:1;
        uint8_t resv:5;
        uint8_t int_enable:1;
    } control2;
    uint8_t reset_period;           // 0x01 as recomemded, read/write
    uint8_t chip_id;                // read only
}Qmc5883lReg;

typedef struct Qmc5883lAxise {
    float x;
    float y;
    float z;
}Qmc5883lAxise;

typedef struct Qmc5883l_ {
    Qmc5883lReg reg;
    uint8_t addr;
    Qmc5883lRate sample_rate;
    Qmc5883lOverSampleRatio ov_ratio;
    Qmc5883lMode mode;
    Qmc5883lRange range;
    Qmc5883lAxise axise;
    float temperature;
    int32_t (*i2c_read)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
    int32_t (*i2c_write)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
    bool inited;
}Qmc5883l;

void Qmc5883l_Register(Qmc5883l *qmc5883l, int32_t (*read)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len), \
                int32_t (*write)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len));
bool Qmc5883l_Init(Qmc5883l *qmc5883l);
bool Qmc5883l_Set(Qmc5883l *qmc5883l, Qmc5883lCmd cmd, uint8_t data);
int32_t Qmc5883l_Process(Qmc5883l *qmc5883l);
int32_t Qmc5883l_Read(Qmc5883l *qmc5883l, Qmc5883lAxise *axise);

#ifdef __cplusplus
}
#endif
#endif