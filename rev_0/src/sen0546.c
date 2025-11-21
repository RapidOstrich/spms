/**
 * @file
 * @brief SEN0546 / CHT832X temperature and humidity one-shot reader.
 *
 * Implements a simple one-shot measurement command (0x2C 0x06),
 * reads back 16-bit temperature and humidity values, and returns them
 * in raw form. Conversion to engineering units (°C, %RH) is handled
 * by the caller using the datasheet formulas.
 */

#include "sen0546.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(SEN0546, LOG_LEVEL_INF);

int query_sen0546(const struct i2c_dt_spec *i2c,
                  uint16_t *temp_raw,
                  uint16_t *rh_raw)
{
    uint8_t cmd[2]     = { 0x2C, 0x06 }; /* One-shot measurement command */
    uint8_t read_buf[6];
    int ret;

    if (!device_is_ready(i2c->bus)) {
        LOG_ERR("SEN0546: I2C bus %s not ready!", i2c->bus->name);
        return -ENODEV;
    }

    /* Write opcode, then read 6 bytes (T_hi, T_lo, T_CRC, RH_hi, RH_lo, RH_CRC). */
    ret = i2c_write_read_dt(i2c, cmd, sizeof(cmd),
                            read_buf, sizeof(read_buf));
    if (ret < 0) {
        LOG_ERR("SEN0546: i2c_write_read_dt failed (addr=0x%02x, err=%d)",
                i2c->addr, ret);
        return ret;
    }

    /* Extract raw data; CRC bytes are ignored. */
    *temp_raw = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    *rh_raw   = ((uint16_t)read_buf[3] << 8) | read_buf[4];

    return 0;
}
