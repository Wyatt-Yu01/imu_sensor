/**
 * @file adlx345.c
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#include "adlx345.h"

void Adlx345_Init(Adlx345_t *m)
{
    if (m && m->read && m->write)
    {
        uint8_t dev_id;
        if (m->read(m, ADLX345_REG_DEVID, &dev_id, 1))     // check device id
        {
            // set resolution & range
            Adlx345RegDataFormat_t format;
            m->read(m, ADLX345_REG_DATAFORMAT, (uint8_t *)&format, 1);
            if (m->fix_resolution)                                      // no use fix resolution
            {
                format.full_res = 1;
                m->full_scale_rate = 1024.0 / 4.0;
                switch(m->range)
                {
                    case Adlx345Range_2g:
                        m->full_scale_rate = 1024.0 / 4.0;
                        break;
                    case Adlx345Range_4g:
                        m->full_scale_rate = 1024.0 / 8.0;
                        break;
                    case Adlx345Range_8g:
                        m->full_scale_rate = 1024.0 / 16.0;
                        break;
                    case Adlx345Range_16g:
                        m->full_scale_rate = 1024.0 / 32.0;
                        break;
                    default:
                        // do nothing
                }
            }
            else
            {
                format.full_res = 0;
                m->full_scale_rate = 1024.0 / 2.0;
            }
            format.range = m->range;

            // set sample rate
            Adlx345RegBwRate_t rate = {0};      // disable low power mode as default
            rate.rate = m->sample_rate;

            if (m->write(m, ADLX345_REG_DATAFORMAT, (uint8_t *)&format, 1) && m->write(m, ADLX345_REG_BW_RATE, (uint8_t *)&rate, 1))
            {
                m->inited = true;
            }
            else
            {
                m->inited = false;
            }
        }
        else
        {
            m->inited = false;
        }
    }
}

void Adlx345_Register(Adlx345_t *m, adlx345_func read, adlx345_func write)
{   
    if (m && read && write)
    {
        m->read = read;
        m->write = write;
    }
}

void Adlx345_Read(Adlx345_t *m, float *x, float *y, float *z)
{
    if (m && m->inited)
    {
        uint8_t raw_bytes[6];
        m->read(m, ADLX345_REG_DATA, raw_bytes, 6);
        m->raw_data[0] = raw_bytes[0] << 8 | raw_bytes[1];
        m->raw_data[1] = raw_bytes[2] << 8 | raw_bytes[3];
        m->raw_data[2] = raw_bytes[4] << 8 | raw_bytes[5];

        m->x = m->raw_data[0] / m->full_scale_rate; 
        m->y = m->raw_data[1] / m->full_scale_rate; 
        m->z = m->raw_data[2] / m->full_scale_rate; 
        *x = m->x;
        *y = m->y;
        *z = m->z;
    }
    else
    {
        // do nothing, value no change
    }
}

void Adlx345_GetSampleRate(Adlx345_t *m, Adlx345SampleRate *sample_rate)
{
    if (m && sample_rate)
    {
        *sample_rate = m->sample_rate;
    }
}

