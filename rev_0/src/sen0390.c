#include "sen0390.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sen0390, LOG_LEVEL_INF);

int query_sen0390(const struct i2c_dt_spec *i2c, uint32_t *lux_raw)
{
    uint8_t reg_addr = 0x00;     /* start of 32-bit illumination registers 0x00–0x03 */
    uint8_t read_buf[4];
    int ret;

    if (lux_raw == NULL) {
        LOG_ERR("SEN0390: lux_raw pointer is NULL");
        return -EINVAL;
    }

    if (!device_is_ready(i2c->bus)) {
        LOG_ERR("SEN0390: I2C bus %s not ready!", i2c->bus->name);
        return -ENODEV;
    }

    /* Set register pointer to 0x00, then read 4 bytes in one transaction */
    ret = i2c_write_read_dt(i2c, &reg_addr, 1, read_buf, sizeof(read_buf));
    if (ret < 0) {
        LOG_ERR("SEN0390: i2c_write_read_dt failed: addr=0x%02x err=%d",
                i2c->addr, ret);
        return ret;
    }

    /* Optional: raw byte debug (only visible if level >= LOG_LEVEL_DBG) */
    LOG_DBG("SEN0390 raw bytes: %02x %02x %02x %02x",
            read_buf[0], read_buf[1], read_buf[2], read_buf[3]);

    *lux_raw = ((uint32_t)read_buf[0] << 24) |
               ((uint32_t)read_buf[1] << 16) |
               ((uint32_t)read_buf[2] <<  8) |
               ((uint32_t)read_buf[3] <<  0);

    return 0;
}
