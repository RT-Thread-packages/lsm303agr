
#include "st_lsm303agr_sensor_v1.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.st.lsm303agr.acce"
#define DBG_COLOR
#include <rtdbg.h>

#define SENSOR_ACC_RANGE_2G   2000
#define SENSOR_ACC_RANGE_4G   4000
#define SENSOR_ACC_RANGE_8G   8000
#define SENSOR_ACC_RANGE_16G  16000

static LSM303AGR_ACC_Object_t acc;
static struct rt_i2c_bus_device *i2c_bus_dev;

static int32_t i2c_init(void)
{
    return 0;
}

static int32_t lsm303agr_get_tick(void)
{
    return rt_tick_get();
}

static int32_t rt_i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int32_t rt_i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _lsm303agr_acc_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t  id, i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;
    LSM303AGR_IO_t io_ctx;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* Configure the accelero driver */
    io_ctx.BusType     = LSM303AGR_I2C_BUS; /* I2C */
    io_ctx.Address     = i2c_addr;
    io_ctx.Init        = i2c_init;
    io_ctx.DeInit      = i2c_init;
    io_ctx.ReadReg     = rt_i2c_read_reg;
    io_ctx.WriteReg    = rt_i2c_write_reg;
    io_ctx.GetTick     = lsm303agr_get_tick;

    if (LSM303AGR_ACC_RegisterBusIO(&acc, &io_ctx) != LSM303AGR_OK)
    {
        return -RT_ERROR;
    }
    else if (LSM303AGR_ACC_ReadID(&acc, &id) != LSM303AGR_OK)
    {
        rt_kprintf("read id failed\n");
        return -RT_ERROR;
    }
    if (LSM303AGR_ACC_Init(&acc) != LSM303AGR_OK)
    {
        rt_kprintf("acc init failed\n");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _lsm303agr_acc_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    if (range <= SENSOR_ACC_RANGE_2G)
    {
        LSM303AGR_ACC_SetFullScale(&acc, 2);
    }
    else if (range > SENSOR_ACC_RANGE_2G && range <= SENSOR_ACC_RANGE_4G)
    {
        LSM303AGR_ACC_SetFullScale(&acc, 4);
    }
    else if (range > SENSOR_ACC_RANGE_4G && range <= SENSOR_ACC_RANGE_8G)
    {
        LSM303AGR_ACC_SetFullScale(&acc, 8);
    }
    else if (range > SENSOR_ACC_RANGE_8G && range <= SENSOR_ACC_RANGE_16G)
    {
        LSM303AGR_ACC_SetFullScale(&acc, 16);
    }
    else
    {
        return -RT_ERROR;
    }
    LOG_D("set range %d", range);
    return RT_EOK;
}

static rt_err_t _lsm303agr_acc_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    LSM303AGR_ACC_SetOutputDataRate(&acc, odr);
    LOG_D("set odr %d", odr);
    return RT_EOK;
}

static rt_err_t _lsm303agr_acc_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    if (mode == RT_SENSOR_MODE_POLLING)
    {
        /* disable fifo */
        lsm303agr_xl_fifo_set(&acc.Ctx, 0);
        /* FIFO mode selection */
        lsm303agr_xl_fifo_mode_set(&acc.Ctx, LSM303AGR_BYPASS_MODE);
        LOG_D("set mode to POLLING");
    }
    else if (mode == RT_SENSOR_MODE_INT)
    {
        LOG_D("set mode to RT_SENSOR_MODE_INT");
    }
    else if (mode == RT_SENSOR_MODE_FIFO)
    {
        lsm303agr_ctrl_reg3_a_t value;

        /* enable fifo */
        lsm303agr_xl_fifo_set(&acc.Ctx, 1);
        /* set fifo mode */
        lsm303agr_xl_fifo_mode_set(&acc.Ctx, LSM303AGR_FIFO_MODE);
        /* enable fifo overrun */
        lsm303agr_xl_pin_int1_config_get(&(acc.Ctx), &value);
        value.i1_overrun = 1;
        lsm303agr_xl_pin_int1_config_set(&(acc.Ctx), &value);

        LOG_D("set mode to RT_SENSOR_MODE_FIFO");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", mode);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _lsm303agr_acc_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        LSM303AGR_ACC_Disable(&acc);
        LOG_D("set power down");
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        LSM303AGR_ACC_Enable(&acc);
        LOG_D("set power normal");
    }
    else
    {
        LOG_D("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _lsm303agr_acc_clear_int(struct rt_sensor_device *sensor)
{
    if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        _lsm303agr_acc_set_mode(sensor, RT_SENSOR_MODE_POLLING);
        _lsm303agr_acc_set_mode(sensor, RT_SENSOR_MODE_FIFO);
    }
    return 0;
}

static rt_size_t _lsm303agr_acc_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    LSM303AGR_Axes_t acceleration;

    LSM303AGR_ACC_GetAxes(&acc, &acceleration);

    data->type = RT_SENSOR_CLASS_ACCE;
    data->data.acce.x = acceleration.x;
    data->data.acce.y = acceleration.y;
    data->data.acce.z = acceleration.z;
    data->timestamp = rt_sensor_get_ts();

    return 1;
}

static rt_size_t _lsm303agr_acc_fifo_get_data(rt_sensor_t sensor, struct rt_sensor_data *data, rt_size_t len)
{
    LSM303AGR_Axes_t acceleration;
    rt_uint8_t i;

    if (len > 32)
    {
        len = 32;
    }

    for (i = 0; i < len; i++)
    {
        if (LSM303AGR_ACC_GetAxes(&acc, &acceleration) == 0)
        {
            data[i].type = RT_SENSOR_CLASS_ACCE;
            data[i].data.acce.x = acceleration.x;
            data[i].data.acce.y = acceleration.y;
            data[i].data.acce.z = acceleration.z;
            data[i].timestamp = rt_sensor_get_ts();
        }
        else
            break;
    }

    _lsm303agr_acc_clear_int(sensor);

    return i;
}

static RT_SIZE_TYPE lsm303agr_acc_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _lsm303agr_acc_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        return 0;
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        return _lsm303agr_acc_fifo_get_data(sensor, buf, len);
    }
    else
        return 0;
}

static rt_err_t lsm303agr_acc_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        LSM303AGR_ACC_ReadID(&acc, args);
        break;
    case RT_SENSOR_CTRL_GET_INFO:
    {
        struct rt_sensor_info *info = args;
        info->vendor = RT_SENSOR_VENDOR_STM;
        info->model  = "lsm303agr_acc";
        info->unit   = RT_SENSOR_UNIT_MGAUSS;
        info->intf_type   = RT_SENSOR_INTF_I2C;
        info->range_max = SENSOR_ACC_RANGE_16G;
        info->range_min = SENSOR_ACC_RANGE_2G;
        info->period_min = 100;
    }
    break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _lsm303agr_acc_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _lsm303agr_acc_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = _lsm303agr_acc_set_mode(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _lsm303agr_acc_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    lsm303agr_acc_fetch_data,
    lsm303agr_acc_control
};

int rt_hw_lsm303agr_acc_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_ACCE;
    sensor->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor->info.model      = "lsm303agr_acc";
    sensor->info.unit       = RT_SENSOR_UNIT_MG;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = SENSOR_ACC_RANGE_16G;
    sensor->info.range_min  = SENSOR_ACC_RANGE_2G;
    sensor->info.period_min = 100;
    sensor->info.fifo_max   = 32;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = _lsm303agr_acc_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("_lsm6dsl acc init err code: %d", result);
        goto __exit;
    }

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_FIFO_RX, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        goto __exit;
    }

    LOG_I("acce sensor init success");
    return RT_EOK;

__exit:
    if (sensor)
        rt_free(sensor);
    return -RT_ERROR;
}
