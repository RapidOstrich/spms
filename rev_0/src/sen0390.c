#include "sen0390.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sen0390, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* Lux calibration (raw -> estimated lux)                                    */
/* ------------------------------------------------------------------------- */
/*
 * TEMPORARY calibration:
 *   Tune these after you measure R_A / R_B in real conditions.
 */
static const int32_t  L_A = 10;       /* approx lux at low reference */
static const int32_t  L_B = 20000;    /* approx lux at high reference */
static const uint32_t R_A = 1100U;    /* ~your current dim-room readings */
static const uint32_t R_B = 50000U;   /* placeholder, tune after bright measurement */

int query_sen0390(const struct i2c_dt_spec *i2c, uint32_t *lux_raw)
{
    uint8_t reg_addr = 0x00;     /* start of illumination registers 0x00–0x03 */
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

    /* Set register pointer to 0x00, then read 4 bytes */
    ret = i2c_write_read_dt(i2c, &reg_addr, 1, read_buf, sizeof(read_buf));
    if (ret < 0) {
        LOG_ERR("SEN0390: i2c_write_read_dt failed: addr=0x%02x err=%d",
                i2c->addr, ret);
        return ret;
    }

    LOG_DBG("Raw bytes: %02x %02x %02x %02x",
            read_buf[0], read_buf[1], read_buf[2], read_buf[3]);

    /*
     * Correct byte order:
     *   0x00 = LSB   (D07..D00)
     *   0x01 =       (D15..D08)
     *   0x02 =       (D23..D16)
     *   0x03 = MSB   (D31..D24)
     */
    *lux_raw = ((uint32_t)read_buf[0] <<  0) |
               ((uint32_t)read_buf[1] <<  8) |
               ((uint32_t)read_buf[2] << 16) |
               ((uint32_t)read_buf[3] << 24);

    LOG_DBG("Combined lux_raw = %u", (unsigned int)*lux_raw);

    return 0;
}

int32_t sen0390_raw_to_lux(uint32_t lux_raw)
{
    /* Guard against division by zero / bogus calibration */
    if (R_B <= R_A) {
        return 0;
    }

    if (lux_raw <= R_A) {
        return L_A;
    }
    if (lux_raw >= R_B) {
        return L_B;
    }

    int64_t num   = (int64_t)(lux_raw - R_A) * (L_B - L_A);
    int64_t denom = (int64_t)(R_B - R_A);

    return (int32_t)(L_A + num / denom);
}