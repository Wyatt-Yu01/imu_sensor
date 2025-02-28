/**
 * @file qmc5883l.c
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#include "qmc5883l.h"

void Qmc5883l_Register(Qmc5883l *qmc5883l, Qmc5883l_I2cMemFunc read, Qmc5883l_I2cMemFunc write)
{
    if (qmc5883l && read && write)
    {
        qmc5883l->read = read;
        qmc5883l->write = write;
    }
    else
    {
        //
    }
}

bool Qmc5883l_Init(Qmc5883l *qmc5883l)
{
    if (qmc5883l && qmc5883l->read && qmc5883l->write)
    {
        if (Qmc5883l_Set(qmc5883l, Qmc5883lCmd_ChipId, 0))  // chip available
        {
            Qmc5883l_Set(qmc5883l, Qmc5883lCmd_ResetPeriod, 0x01);   

            uint8_t control1 = (qmc5883l->ov_ratio << 6) | (qmc5883l->range << 4) | \
                                (qmc5883l->sample_rate << 2) | qmc5883l->mode;
            qmc5883l->write(QMC5883L_ADDR, 0x09, &control1, 1);

            uint8_t control2 = 0x40;
            qmc5883l->write(QMC5883L_ADDR, 0x0A, &control2, 1); 
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

// something wrong
bool Qmc5883l_Set(Qmc5883l *qmc5883l, Qmc5883lCmd cmd, uint8_t data)
{
    if (qmc5883l->read && qmc5883l->write)
    {
        int32_t ret = false;
        switch (cmd)
        {
        case Qmc5883lCmd_Mode:
            qmc5883l->reg.control.mode = data;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_DataRate:
            qmc5883l->reg.control.odr = data;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_FullScale:
            qmc5883l->reg.control.rng = data;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_OverSampleRate:
            qmc5883l->reg.control.osr = data;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x09, (uint8_t *)&qmc5883l->reg.control, 1);
            break;
        case Qmc5883lCmd_Reset:
            qmc5883l->reg.control2.soft_reset = data;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x0A, (uint8_t *)&qmc5883l->reg.control2, 1);
            break;
        case Qmc5883lCmd_ResetPeriod:
            qmc5883l->reg.reset_period = data;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x0B, (uint8_t *)&qmc5883l->reg.reset_period, 1);
            break;
        case Qmc5883lCmd_RolPnt:
            qmc5883l->reg.control2.rol_pnt = data ? 1 : 0;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x0A, (uint8_t *)&qmc5883l->reg.control2, 1);
            break;
        case Qmc5883lCmd_InterruptEnable:
            qmc5883l->reg.control2.int_enable = data ? 1 : 0;
            ret = qmc5883l->write(QMC5883L_ADDR, 0x0A, (uint8_t *)&qmc5883l->reg.control2, 1);
            break;
        case Qmc5883lCmd_ChipId:
            ret = qmc5883l->read(QMC5883L_ADDR, 0x0D, (uint8_t *)&qmc5883l->reg.chip_id, 1);
            break;
        case Qmc5883lCmd_ReadStatus:
            ret = qmc5883l->read(QMC5883L_ADDR, 0x06, (uint8_t *)&qmc5883l->reg.status, 8);
            break;
        default:
            ret = false;
            break;
        }
        return ret;
    }
    else 
    {
        return false;
    }
}

bool Qmc5883l_Read(Qmc5883l *qmc5883l, Qmc5883lAxes *axes)
{
    bool ret = false;
    if (qmc5883l && qmc5883l->inited)
    {
        uint8_t buffer[6] = {0};
        qmc5883l->read(QMC5883L_ADDR, 0, buffer, 6);
        qmc5883l->raw_data[0] = buffer[0] | (buffer[1] << 8);
        qmc5883l->raw_data[1] = buffer[2] | (buffer[3] << 8);
        qmc5883l->raw_data[2] = buffer[4] | (buffer[5] << 8);

        float sensitivity = 12000.0;        // sensitivity defined in datasheet
        switch (qmc5883l->range)
        {
            case Qmc5883lRange_2gauss:
                sensitivity = 12000.0;
                break;
            case Qmc5883lRange_8gauss:
                sensitivity = 3000.0;
                break;
            case Qmc5883lRange_reserve:
                sensitivity = 12000.0;
                break;
            default:
                break;
        }
        qmc5883l->axes.x = (buffer[0] | (buffer[1] << 8)) / sensitivity;
        qmc5883l->axes.y = (buffer[2] | (buffer[3] << 8)) / sensitivity;
        qmc5883l->axes.z = (buffer[4] | (buffer[5] << 8)) / sensitivity;
        ret = true;
    }
    else
    {
        ret = false;
    }
    axes->x = qmc5883l->axes.x;
    axes->y = qmc5883l->axes.y;
    axes->z = qmc5883l->axes.z;
    return ret;
}
