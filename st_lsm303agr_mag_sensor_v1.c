
#include "st_lsm303agr_sensor_v1.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.st.lsm303agr.mag"
#define DBG_COLOR
#include <rtdbg.h>

static LSM303AGR_MAG_Object_t mag;
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

static rt_err_t _lsm303agr_mag_init(struct rt_sensor_intf *intf)
{
    LSM303AGR_IO_t io_ctx;
    rt_uint8_t        id;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* Configure the maglero driver */
    io_ctx.BusType     = LSM303AGR_I2C_BUS; /* I2C */
    io_ctx.Address     = (rt_uint32_t)(intf->user_data) & 0xff;
    io_ctx.Init        = i2c_init;
    io_ctx.DeInit      = i2c_init;
    io_ctx.ReadReg     = rt_i2c_read_reg;
    io_ctx.WriteReg    = rt_i2c_write_reg;
    io_ctx.GetTick     = lsm303agr_get_tick;

    if (LSM303AGR_MAG_RegisterBusIO(&mag, &io_ctx) != LSM303AGR_OK)
    {
        return -RT_ERROR;
    }
    else if (LSM303AGR_MAG_ReadID(&mag, &id) != LSM303AGR_OK)
    {
        LOG_E("read id failed");
        return -RT_ERROR;
    }
    if (LSM303AGR_MAG_Init(&mag) != LSM303AGR_OK)
    {
        LOG_E("mag init failed");
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t _lsm303agr_mag_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        LOG_D("set power down");
        LSM303AGR_MAG_Disable(&mag);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        LOG_D("set power normal");
        LSM303AGR_MAG_Enable(&mag);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

static RT_SIZE_TYPE lsm303agr_mag_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    LSM303AGR_Axes_t magleration;

    RT_ASSERT(buf);

    LSM303AGR_MAG_GetAxes(&mag, &magleration);

    data->type = RT_SENSOR_CLASS_MAG;
    data->data.mag.x = magleration.x;
    data->data.mag.y = magleration.y;
    data->data.mag.z = magleration.z;
    data->timestamp = rt_sensor_get_ts();

    return 1;
}

static rt_err_t lsm303agr_mag_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        LSM303AGR_MAG_ReadID(&mag, args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
    {
        rt_uint16_t odr = (rt_uint32_t)args & 0xffff;
        LOG_D("set odr %d", odr);
        LSM303AGR_MAG_SetOutputDataRate(&mag, odr);
    }
    break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _lsm303agr_mag_set_power(sensor, (rt_uint32_t)args & 0xff);
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
    lsm303agr_mag_fetch_data,
    lsm303agr_mag_control
};

int rt_hw_lsm303agr_mag_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_MAG;
    sensor->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor->info.model      = "lsm303agr_mag";
    sensor->info.unit       = RT_SENSOR_UNIT_MGAUSS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 50000;
    sensor->info.range_min  = 50000;
    sensor->info.period_min = 100;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;

    result = _lsm303agr_mag_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("_lsm6dsl acc init err code: %d", result);
        goto __exit;
    }

    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
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
