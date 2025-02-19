/**
 * @file qmc5883l.c
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */
 
#include "qmc5883l.h"

void Qmc5883l_Register(Qmc5883l *qmc5883l, int32_t (*read)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len), \
                    int32_t (*write)(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len))
{
    if (qmc5883l && read && write)
    {
        qmc5883l->i2c_read = read;
        qmc5883l->i2c_write = write;
    }
    else
    {
        //
    }
}

bool Qmc5883l_Init(Qmc5883l *qmc5883l)
{
    if (qmc5883l && qmc5883l->i2c_read && qmc5883l->i2c_write)
    {
        if (Qmc5883l_Set(qmc5883l, Qmc5883lCmd_ReadStatus, 0))  // chip avilable
        {
            qmc5883l->reg.control.mode = qmc5883l->mode;
            qmc5883l->reg.control.osr = qmc5883l->ov_ratio;
            qmc5883l->reg.control.rng = qmc5883l->range;
            qmc5883l->reg.control.odr = qmc5883l->sample_rate;
            qmc5883l->reg.control2.rol_pnt = 0;                 // 关闭数据寄存器自动滚动保存
            qmc5883l->reg.control2.int_enable = 0;            // 关闭中断引脚输出功能 PIN DRDY
            Qmc5883l_Set(qmc5883l, Qmc5883lCmd_Mode, (uint8_t)qmc5883l->mode);
            qmc5883l->inited = true;
            return true;
        }
        else 
        {
            return false;
        }
    }
    else 
    {
        return false;
    }
}

bool Qmc5883l_Set(Qmc5883l *qmc5883l, Qmc5883lCmd cmd, uint8_t data)
{
    if (qmc5883l->inited && qmc5883l->i2c_read && qmc5883l->i2c_write)
    {
        int32_t ret = true;
        switch (cmd)
        {
        case Qmc5883lCmd_Mode:
            qmc5883l->reg.control.mode = data;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_DataRate:
            qmc5883l->reg.control.odr = data;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_FullScall:
            qmc5883l->reg.control.rng = data;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_OverSampleRate:
            qmc5883l->reg.control.osr = data;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_Reset:
            qmc5883l->reg.control2.soft_reset = data;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x0A, (uint8_t *)&qmc5883l->reg.control2, 1);
            break;
        case Qmc5883lCmd_ResetPeriod:
            qmc5883l->reg.reset_period = data;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x0B, (uint8_t *)&qmc5883l->reg.reset_period, 1);
            break;
        case Qmc5883lCmd_RolPnt:
            qmc5883l->reg.control2.rol_pnt = data ? 1 : 0;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x0A, (uint8_t *)&qmc5883l->reg.control2, 1);
            break;
        case Qmc5883lCmd_InterruptEnable:
            qmc5883l->reg.control2.int_enable = data ? 1 : 0;
            ret = qmc5883l->i2c_write(qmc5883l->addr, 0x0A, (uint8_t *)&qmc5883l->reg.control2, 1);
            break;
        case Qmc5883lCmd_ChipId:
            ret = qmc5883l->i2c_read(qmc5883l->addr, 0x0D, (uint8_t *)&qmc5883l->reg.chip_id, 1);
            break;
        case Qmc5883lCmd_ReadStatus:
            ret = qmc5883l->i2c_read(qmc5883l->addr, 0x06, (uint8_t *)&qmc5883l->reg.status, 8);
            break;
        default:
            ret = 0;
            break;
        }
        return  (ret > 0) ? true : false;
    }
    else 
    {
        return false;
    }
}

int32_t Qmc5883l_Process(Qmc5883l *qmc5883l)
{
    if (qmc5883l && qmc5883l->inited)
    {
        int16_t magicx, magicy, magicz;
        qmc5883l->i2c_read(qmc5883l->addr, 0x00, (uint8_t *)&qmc5883l->reg, 6);
        magicx = qmc5883l->reg.x_msb << 8 | qmc5883l->reg.x_lsb;
        magicy = qmc5883l->reg.y_msb << 8 | qmc5883l->reg.y_lsb;
        magicz = qmc5883l->reg.z_msb << 8 | qmc5883l->reg.z_lsb;

        float magic_range = 2.0;
        switch (qmc5883l->range)
        {
            case Qmc5883lRange_2gauss:
                magic_range = 2.0;
                break;
            case Qmc5883lRange_8gauss:
                magic_range = 8.0;
                break;
            case Qmc5883lRange_reserve:
                magic_range = 2.0;
                break;
        }
        qmc5883l->axise.x = magicx * magic_range / 32768.0f;
        qmc5883l->axise.y = magicy * magic_range / 32768.0f;
        qmc5883l->axise.z = magicz * magic_range / 32768.0f;
    }
    else
    {
        // do nothing
    }
}

int32_t Qmc5883l_Read(Qmc5883l *qmc5883l, Qmc5883lAxise *axise)
{
    axise->x = qmc5883l->axise.x;
    axise->y = qmc5883l->axise.y;
    axise->z = qmc5883l->axise.z;
}
