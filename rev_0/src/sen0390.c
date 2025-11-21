/**
 * @file
 * @brief SEN0390 ambient light sensor implementation.
 *
 * Provides raw 32-bit light readings via I2C and applies a simple
 * linear two-point calibration (defined in this file) to estimate
 * illuminance in lux. The caller may refine calibration constants
 * after collecting real measurements.
 */

#include "sen0390.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(SEN0390, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* Calibration constants (placeholder — tune with real measurements)         */
/* ------------------------------------------------------------------------- */

/*
 * L_A / L_B are lux estimates at reference points.
 * R_A / R_B are corresponding raw counts measured at those lux levels.
 *
 * These initial values produce a reasonable mapping shape for development.
 * Replace with measured values once you have a proper lux meter.
 */
static const int32_t  L_A = 10;        /* Low-light lux reference */
static const int32_t  L_B = 20000;     /* High-light lux reference */

static const uint32_t R_A = 1100U;     /* Raw count at low light */
static const uint32_t R_B = 50000U;    /* Raw count at high light */

/* Maximum valid raw value (device uses up to 24 bits) */
#define SEN0390_RAW_MAX_VALID  (0x00FFFFFFU)

/* ------------------------------------------------------------------------- */
/* Raw I2C reading                                                           */
/* ------------------------------------------------------------------------- */

int query_sen0390(const struct i2c_dt_spec *i2c, uint32_t *lux_raw)
{
    uint8_t reg_addr = 0x00;   /* Start of 4-byte illumination register block */
    uint8_t buf[4];
    int ret;

    if (lux_raw == NULL) {
        LOG_ERR("SEN0390: lux_raw pointer is NULL");
        return -EINVAL;
    }

    if (!device_is_ready(i2c->bus)) {
        LOG_ERR("SEN0390: I2C bus %s not ready!", i2c->bus->name);
        return -ENODEV;
    }

    /* Write starting register address, then read 4 bytes. */
    ret = i2c_write_read_dt(i2c, &reg_addr, 1, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("SEN0390: i2c_write_read_dt failed: addr=0x%02x err=%d",
                i2c->addr, ret);
        return ret;
    }

    LOG_DBG("SEN0390 raw bytes: %02x %02x %02x %02x",
            buf[0], buf[1], buf[2], buf[3]);

    /*
     * Byte layout:
     *   buf[0] = LSB (D07..D00)
     *   buf[1] =       (D15..D08)
     *   buf[2] =       (D23..D16)
     *   buf[3] = MSB   (D31..D24)
     *
     * Device uses only 24 bits; upper byte is typically 0.
     */
    *lux_raw = ((uint32_t)buf[0] << 0)  |
               ((uint32_t)buf[1] << 8)  |
               ((uint32_t)buf[2] << 16) |
               ((uint32_t)buf[3] << 24);

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Calibration: raw -> estimated lux                                         */
/* ------------------------------------------------------------------------- */

int32_t sen0390_raw_to_lux(uint32_t lux_raw)
{
    /* Clamp invalid values */
    if (lux_raw > SEN0390_RAW_MAX_VALID) {
        lux_raw = SEN0390_RAW_MAX_VALID;
    }

    /*
     * 2-point linear interpolation:
     *     y = L_A + (L_B - L_A) * (x - R_A) / (R_B - R_A)
     */
    int32_t num   = (int32_t)lux_raw - (int32_t)R_A;
    int32_t denom = (int32_t)R_B - (int32_t)R_A;

    if (denom == 0) {
        return L_A; /* Avoid divide-by-zero; degenerate calibration */
    }

    int32_t est = L_A + ((L_B - L_A) * num) / denom;

    if (est < 0) {
        est = 0;
    }

    return est;
}
