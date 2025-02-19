/**
 * @file itg3205.c
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#include "itg3205.h"

#define ITG3205_DEGREE2RAD(x) (x * 3.1415926535 / 180.0)

void Itg3205_Init(Itg3205_t *m)
{
    if (m && m->read && m->write)
    {
        uint8_t device_id = 0;
        if (m->read(m, ITG3205_REG_DEVID, &device_id, 1))
        {
            m->write(m, ITG3205_REG_DLPF, &m->lpf, 1);
            m->write(m, ITG3205_REG_SAMPLE_RATE_DIV, &m->sample_div, 1);
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

void Itg3205_Register(Itg3205_t *m, Itg3205_func read, Itg3205_func write)
{
    if (m && read && write)
    {
        m->read = read;
        m->write = write;
    }
}

void Itg3205_Read(Itg3205_t *m, float *temp, float *x, float *y, float *z)
{
    if (m && m->inited)
    {
        uint8_t data[8];
        m->read(m, ITG3205_REG_DATA, data, 8);
        m->raw_data[0] = data[0] << 8 | data[1];
        m->raw_data[1] = data[2] << 8 | data[3];
        m->raw_data[2] = data[4] << 8 | data[5];
        m->raw_data[3] = data[6] << 8 | data[7];

        // 以下常量是数据手册定义的
        m->temp = (m->raw_data[0] - 13200) / 280.0 + 35.0;
        m->x = ITG3205_DEGREE2RAD((m->raw_data[1] - 32767) / 14.376);
        m->y = ITG3205_DEGREE2RAD((m->raw_data[2] - 32767) / 14.376);
        m->z = ITG3205_DEGREE2RAD((m->raw_data[3] - 32767) / 14.376);

        *temp = m->temp;
        *x = m->x;
        *y = m->y;
        *z = m->z;
    }
}

int32_t Itg3205_GetSampleRate(Itg3205_t *m)
{
    return m->sample_rate;
}
