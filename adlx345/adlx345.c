/**
 * @file adlx345.c
 * @author Wyatt Yu
 * @brief 
 * @copyright Copyright (c) 2025
 */

#include "adlx345.h"
#include "rtdevice.h"
#define DBG_SECTION_NAME "adlx"
#define DBG_COLOR
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>

#define ADLX345_GRAVITY                 (9.81)

void Adlx345_Register(Adlx345 *m, Adlx345_I2cMemFunc read, Adlx345_I2cMemFunc write)
{   
    if (m && read && write)
    {
        m->read = read;
        m->write = write;
    }
}

void Adlx345_Init(Adlx345 *m)
{
    if (m && m->read && m->write)
    {
        uint8_t dev_id;
        if (m->read(m->addr, ADLX345_REG_DEVID, &dev_id, 1))     // check device id
        {
            // set resolution & range
            Adlx345RegDataFormat format;
            m->read(m->addr, ADLX345_REG_DATAFORMAT, (uint8_t *)&format, 1);
            if (m->fix_resolution)                                      // no use fix resolution
            {
                format.full_res = 0;
                
                switch(m->range)
                {
                    case Adlx345Range_2g:
                        m->full_scale_rate = 1024.0 * 1024.0 / (4.0 * ADLX345_GRAVITY);
                        break;
                    case Adlx345Range_4g:
                        m->full_scale_rate = 1024.0 * 1024.0 / (8.0 * ADLX345_GRAVITY);
                        break;
                    case Adlx345Range_8g:
                        m->full_scale_rate = 1024.0 * 1024.0 / (16.0 * ADLX345_GRAVITY);
                        break;
                    case Adlx345Range_16g:
                        m->full_scale_rate = 1024.0 * 1024.0 / (32.0 * ADLX345_GRAVITY);
                        break;
                    default:
                        m->full_scale_rate = 1024.0 * 1024.0 / (4.0 * ADLX345_GRAVITY);
                }
            }
            else
            {
                format.full_res = 1;
                switch(m->range)
                {
                    case Adlx345Range_2g:
                        m->full_scale_rate = 64.0 * 256.0 / ADLX345_GRAVITY;
                        break;
                    case Adlx345Range_4g:
                        m->full_scale_rate = 32.0 * 256.0 / ADLX345_GRAVITY;
                        break;
                    case Adlx345Range_8g:
                        m->full_scale_rate = 16.0 * 256.0 / ADLX345_GRAVITY;
                        break;
                    case Adlx345Range_16g:
                        m->full_scale_rate = 8.0 * 256.0 / ADLX345_GRAVITY;
                        break;
                    default:
                        m->full_scale_rate = 64.0 * 256.0 / ADLX345_GRAVITY;
                }
            }
            format.range = m->range;

#if 0
            // set sample rate
            Adlx345RegBwRate bw_rate = {
                .resvd = 0,
                .low_power = 0,             // disable low power
                .rate = m->sample_rate,
            };      // disable low power mode as default
#else
            uint8_t bw_rate = m->sample_rate;
#endif
            uint8_t power_ctrl = 0x08;                  //0B-00-0-0-1-0-00 
            uint8_t data_format = 0x0c | m->range;     // 0B-0-0-0-0-1-1-11     //full-res, left-justify
            m->write(m->addr, ADLX345_REG_POWER_CTL, &power_ctrl, 1);

            if (m->write(m->addr, ADLX345_REG_DATAFORMAT, &data_format, 1) && \
                    m->write(m->addr, ADLX345_REG_BW_RATE, (uint8_t *)&bw_rate, 1))
            {
                LOG_D("init success");
                m->inited = true;
            }
            else
            {
                LOG_D("init failed");
                m->inited = false;
            }
        }
        else
        {
            LOG_D("access chip failed");
            m->inited = false;
        }
    }
    else 
    {
        LOG_D("handler is null");
    }
}

bool Adlx345_Read(Adlx345 *m, Adlx345Axes *axes)
{
    int ret = false;
    if (m && m->inited)
    {
        uint8_t raw_bytes[6];
        if (m->read(m->addr, ADLX345_REG_DATA, raw_bytes, 6))
        {
            m->raw_data[0] = (raw_bytes[1] << 8) | raw_bytes[0];
            m->raw_data[1] = (raw_bytes[3] << 8) | raw_bytes[2];
            m->raw_data[2] = (raw_bytes[5] << 8) | raw_bytes[4];
            m->axes.x = 1.0 * m->raw_data[0];
            m->axes.y = 1.0 * m->raw_data[1];
            m->axes.z = 1.0 * m->raw_data[2];
            m->axes.x /= m->full_scale_rate;
            m->axes.y /= m->full_scale_rate;
            m->axes.z /= m->full_scale_rate; 
            ret = true;
        }
        else
        {   
            LOG_E("read data error");
            ret = false;
        }
        
    }
    else
    {
        ret = false;
    }

    axes->x = m->axes.x;
    axes->y = m->axes.y;
    axes->z = m->axes.z;
    return ret;
}

void Adlx345_GetSampleRate(Adlx345 *m, Adlx345SampleRate *sample_rate)
{
    if (m && sample_rate)
    {
        *sample_rate = m->sample_rate;
    }
}

