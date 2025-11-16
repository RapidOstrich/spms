#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include "sen0390.h"
#include "sen0546.h"
#include "spms_sensors.h"

LOG_MODULE_REGISTER(spms_sensors, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* I2C / Sensor binding                                                      */
/* ------------------------------------------------------------------------- */

#define SEN0390_NODE DT_NODELABEL(sen0390)
#define CHT832X_NODE DT_NODELABEL(cht832x)

#if !DT_NODE_HAS_STATUS(SEN0390_NODE, okay)
#error "SEN0390 node is not okay; check your nrf52dk_nrf52832.overlay"
#endif

#if !DT_NODE_HAS_STATUS(CHT832X_NODE, okay)
#error "CHT832X node is not okay; check your nrf52dk_nrf52832.overlay"
#endif

static const struct i2c_dt_spec sen0390_i2c = I2C_DT_SPEC_GET(SEN0390_NODE);
static const struct i2c_dt_spec sen0546_i2c = I2C_DT_SPEC_GET(CHT832X_NODE);

/* ------------------------------------------------------------------------- */
/* Lux filtering state                                                       */
/* ------------------------------------------------------------------------- */

static uint32_t lux_filtered       = 0;
static bool     lux_filtered_valid = false;

static void update_lux_filtered(uint32_t sample)
{
    if (!lux_filtered_valid) {
        lux_filtered = sample;
        lux_filtered_valid = true;
        return;
    }

    /* Simple IIR filter:
     * Normal case: alpha = 1/4
     * Outlier case (>2x or <1/2): alpha = 1/16
     */
    if (sample > 2U * lux_filtered || sample * 2U < lux_filtered) {
        /* Suspect spike/dip: blend in slowly */
        lux_filtered += (sample - lux_filtered) / 16U;
    } else {
        /* Normal update */
        lux_filtered += (sample - lux_filtered) / 4U;
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

int spms_sensors_init(void)
{
    if (!device_is_ready(sen0390_i2c.bus)) {
        LOG_ERR("I2C bus %s for SEN0390 not ready!",
                sen0390_i2c.bus->name);
        return -ENODEV;
    }

    if (!device_is_ready(sen0546_i2c.bus)) {
        LOG_ERR("I2C bus %s for CHT832X (SEN0546) not ready!",
                sen0546_i2c.bus->name);
        return -ENODEV;
    }

    LOG_INF("Sensors I2C ready: SEN0390 addr=0x%02x, CHT832X addr=0x%02x",
            sen0390_i2c.addr, sen0546_i2c.addr);

    lux_filtered_valid = false;
    lux_filtered       = 0;

    return 0;
}

int spms_sensors_update(const struct plant_profile *profile,
                        struct spms_sensor_snapshot *out)
{
    if (!out) {
        return -EINVAL;
    }

    uint32_t lux_raw_local  = 0;
    uint16_t temp_raw = 0;
    uint16_t rh_raw   = 0;
    int ret;

    ret = query_sen0390(&sen0390_i2c, &lux_raw_local);
    if (ret < 0) {
        LOG_ERR("query_sen0390 failed: %d", ret);
        return ret;
    }

    /* Update raw + filtered lux */
    update_lux_filtered(lux_raw_local);

    /* Give the bus/sensor a short breather before the next query */
    k_sleep(K_MSEC(100));

    ret = query_sen0546(&sen0546_i2c, &temp_raw, &rh_raw);
    if (ret < 0) {
        LOG_ERR("query_sen0546 failed: %d", ret);
        return ret;
    }

    float temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    float rh     = 100.0f * ((float)rh_raw / 65535.0f);

    int16_t  temp_c_x100 = (int16_t)(temp_c * 100.0f);
    uint16_t rh_x100     = (uint16_t)(rh * 100.0f);
    int32_t  lux_est     = sen0390_raw_to_lux(lux_filtered);

    /* Fill snapshot */
    out->temp_c_x100  = temp_c_x100;
    out->rh_x100      = rh_x100;
    out->lux_raw      = lux_raw_local;
    out->lux_filtered = lux_filtered;
    out->lux_est      = lux_est;

    LOG_INF("Sensor: T = %d.%02d °C, RH = %u.%02u %% "
            "LUX(raw) = %u filt = %u est = %ld lux",
            temp_c_x100 / 100,
            (temp_c_x100 >= 0 ? temp_c_x100 % 100 : -(temp_c_x100 % 100)),
            rh_x100 / 100,
            rh_x100 % 100,
            (unsigned int)lux_raw_local,
            (unsigned int)lux_filtered,
            (long)lux_est);

    /* --------- Compare against plant profile ---------------------------- */

    if (profile != NULL) {
        /* Temperature */
        if (temp_c_x100 < profile->temp_min_x100) {
            LOG_WRN("Plant profile: temperature BELOW ideal (min=%d)",
                    profile->temp_min_x100);
        } else if (temp_c_x100 > profile->temp_max_x100) {
            LOG_WRN("Plant profile: temperature ABOVE ideal (max=%d)",
                    profile->temp_max_x100);
        }

        /* Humidity */
        if (rh_x100 < profile->rh_min_x100) {
            LOG_WRN("Plant profile: humidity BELOW ideal (min=%d)",
                    profile->rh_min_x100);
        } else if (rh_x100 > profile->rh_max_x100) {
            LOG_WRN("Plant profile: humidity ABOVE ideal (max=%d)",
                    profile->rh_max_x100);
        }

        /* Lux */
        if (lux_est < profile->lux_min) {
            LOG_WRN("Plant profile: light too LOW (min=%d)", profile->lux_min);
        } else if (lux_est > profile->lux_max) {
            LOG_WRN("Plant profile: light too HIGH (max=%d)", profile->lux_max);
        }

        /* Moisture will be checked once SEN0114 is integrated.
         * For example:
         *
         *   if (moisture_mv < profile->moisture_min_mv) { ... }
         *   else if (moisture_mv > profile->moisture_max_mv) { ... }
         */
    }

    return 0;
}
