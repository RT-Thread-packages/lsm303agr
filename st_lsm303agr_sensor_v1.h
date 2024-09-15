
#ifndef SENSOR_ST_LSM303AGR_H__
#define SENSOR_ST_LSM303AGR_H__

#include <rtthread.h>
#include <rtdevice.h>

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif

#include "lsm303agr.h"

#define LSM303AGR_ACC_ADDR_DEFAULT (0x19)
#define LSM303AGR_MAG_ADDR_DEFAULT (0x1E)

int rt_hw_lsm303agr_acc_init(const char *name, struct rt_sensor_config *cfg);
int rt_hw_lsm303agr_mag_init(const char *name, struct rt_sensor_config *cfg);

#endif
