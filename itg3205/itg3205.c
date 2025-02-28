/**
 * @file itg3205.c
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#include "itg3205.h"

#define ITG3205_DEGREE2RAD(x) ((x) * 3.1415926535 / 180.0)

void Itg3205_Register(Itg3205 *m, Itg3205_I2cMemFunc read, Itg3205_I2cMemFunc write)
{
    if (m && read && write)
    {
        m->read  = read;
        m->write = write;
    }
}

void Itg3205_Init(Itg3205 *m)
{
    if (m && m->read && m->write)
    {
        uint8_t device_id = 0;
        if (m->read(m->addr, ITG3205_REG_DEVID, &device_id, 1))
        {
            m->write(m->addr, ITG3205_REG_DLPF, &m->lpf, 1);
            m->write(m->addr, ITG3205_REG_SAMPLE_RATE_DIV, &m->sample_div, 1);
            switch(m->lpf)
            {
                case Itg3205DlpfBaudrate_256:
                    m->sample_rate = 8000 / (m->sample_div + 1);
                    break;
                default:
                    m->sample_rate = 1000 / (m->sample_div + 1);
            }
            m->inited = true;
        }
        else
        {
            m->inited = false;
        }
    }
}

bool Itg3205_Read(Itg3205 *m, float *temperature, Itg3205Axes *axes)
{
    bool ret = false;
    if (m && m->inited)
    {
        uint8_t data[8];
        m->read(m->addr, ITG3205_REG_DATA, data, 8);
        m->raw_data[0] = (data[0] << 8) | data[1];
        m->raw_data[1] = (data[2] << 8) | data[3];
        m->raw_data[2] = (data[4] << 8) | data[5];
        m->raw_data[3] = (data[6] << 8) | data[7];

        // 以下常量是数据手册定义的
        m->temperature = ((data[0] << 8 | data[1]) - 13200) / 280.0 + 35.0;
        m->axes.x      = ITG3205_DEGREE2RAD(((int16_t)((data[2] << 8) | data[3])) / 14.375);
        m->axes.y      = ITG3205_DEGREE2RAD(((int16_t)((data[4] << 8) | data[5])) / 14.375);
        m->axes.z      = ITG3205_DEGREE2RAD(((int16_t)((data[6] << 8) | data[7])) / 14.375);
        ret = true;
    }
    else 
    {
        ret = false;
    }
    *temperature = m->temperature;
    axes->x      = m->axes.x;
    axes->y      = m->axes.y;
    axes->z      = m->axes.z;
    return ret;
}

int32_t Itg3205_GetSampleRate(Itg3205 *m)
{
    return m->sample_rate;
}
