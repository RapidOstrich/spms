/**
 * @file
 * @brief Unified sensor subsystem for the Smart Plant Monitoring System (SPMS).
 *
 * Binds and initializes all sensors:
 *   - SEN0390 ambient light sensor
 *   - SEN0546 (CHT832X) temperature & humidity sensor
 *   - SEN0114 moisture sensor (ADC-based)
 *
 * Provides:
 *   - spms_sensors_init()  : basic bring-up and sanity checks
 *   - spms_sensors_update(): reads all sensors, updates lux filter,
 *                            computes engineering units, and compares
 *                            against an optional plant profile.
 */

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>

#include "sen0114.h"
#include "sen0390.h"
#include "sen0546.h"
#include "spms_sensors.h"

LOG_MODULE_REGISTER(SPMS_SENSORS, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* I2C / ADC bindings                                                        */
/* ------------------------------------------------------------------------- */

#define SEN0114_NODE DT_PATH(zephyr_user)
#define SEN0390_NODE DT_NODELABEL(sen0390)
#define SEN0546_NODE DT_NODELABEL(sen0546)

#if !DT_NODE_HAS_STATUS(SEN0390_NODE, okay)
#error "SEN0390 node is not okay; check your nrf52dk_nrf52832.overlay"
#endif

#if !DT_NODE_HAS_STATUS(SEN0546_NODE, okay)
#error "SEN0546 node is not okay; check your nrf52dk_nrf52832.overlay"
#endif

static const struct adc_dt_spec  sen0114_adc  = ADC_DT_SPEC_GET(SEN0114_NODE);
static const struct i2c_dt_spec  sen0390_i2c  = I2C_DT_SPEC_GET(SEN0390_NODE);
static const struct i2c_dt_spec  sen0546_i2c  = I2C_DT_SPEC_GET(SEN0546_NODE);

/* ------------------------------------------------------------------------- */
/* Lux filtering state                                                       */
/* ------------------------------------------------------------------------- */

static int32_t lux_filtered       = 0;
static bool    lux_filtered_valid = false;

/**
 * @brief Update the internal low-pass filtered lux value.
 *
 * Simple IIR filter:
 *   - Normal case: alpha = 1/4
 *   - Outlier case (>2x or <1/2 previous value): alpha = 1/16
 */
static void update_lux_filtered(uint32_t sample)
{
    int32_t s = (int32_t)sample;

    /* First valid sample: seed the filter */
    if (!lux_filtered_valid) {
        lux_filtered       = s;
        lux_filtered_valid = true;
        return;
    }

    /* Signed difference from current filtered value */
    int32_t diff = s - lux_filtered;

    /*
     * Simple IIR filter:
     *   - Normal case: alpha = 1/4
     *   - Outlier case (>2x or <1/2): alpha = 1/16
     */
    int32_t abs_diff = (diff >= 0) ? diff : -diff;

    if (abs_diff > (lux_filtered >> 1)) {
        /* Suspected spike/dip: blend in slowly */
        lux_filtered += diff / 16;
    } else {
        /* Normal update */
        lux_filtered += diff / 4;
    }

    /* Clamp to non-negative range just in case */
    if (lux_filtered < 0) {
        lux_filtered = 0;
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
        LOG_ERR("I2C bus %s for SEN0546 not ready!",
                sen0546_i2c.bus->name);
        return -ENODEV;
    }

    LOG_INF("Sensors I2C ready: SEN0390 addr=0x%02x, SEN0546 addr=0x%02x",
            sen0390_i2c.addr, sen0546_i2c.addr);

    int rc = sen0114_init(&sen0114_adc);
    if (rc) {
        LOG_ERR("Moisture: sen0114_init failed (err %d)", rc);
        return rc;
    }

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

    uint32_t lux_raw_local = 0;
    uint16_t temp_raw      = 0;
    uint16_t rh_raw        = 0;
    int      moisture_mv   = 0;
    int      ret;

    /* --------- Ambient light (SEN0390) ----------------------------------- */

    ret = query_sen0390(&sen0390_i2c, &lux_raw_local);
    if (ret < 0) {
        LOG_ERR("query_sen0390 failed: %d", ret);
        return ret;
    }

    /* Update raw + filtered lux */
    update_lux_filtered(lux_raw_local);

    /* Short delay between I2C operations */
    k_sleep(K_MSEC(100));

    /* --------- Temperature & humidity (SEN0546 / CHT832X) ---------------- */

    ret = query_sen0546(&sen0546_i2c, &temp_raw, &rh_raw);
    if (ret < 0) {
        LOG_ERR("query_sen0546 failed: %d", ret);
        return ret;
    }

    float temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    float rh     = 100.0f * ((float)rh_raw / 65535.0f);

    int16_t  temp_c_x100 = (int16_t)(temp_c * 100.0f);
    uint16_t rh_x100     = (uint16_t)(rh * 100.0f);

    /* --------- Lux estimate (calibrated) --------------------------------- */

    /* Use clamped non-negative filtered value for calibration */
    uint32_t lux_filtered_u = (lux_filtered < 0) ? 0U : (uint32_t)lux_filtered;
    int32_t  lux_est        = sen0390_raw_to_lux(lux_filtered_u);

    /* --------- Moisture (SEN0114) ---------------------------------------- */

    ret = query_sen0114_mv(&sen0114_adc, &moisture_mv);
    if (ret < 0) {
        LOG_ERR("Moisture: query_sen0114_mv failed (err %d)", ret);
        return ret;
    }

    /* --------- Fill snapshot ---------------------------------------------- */

    out->temp_c_x100  = temp_c_x100;
    out->rh_x100      = rh_x100;
    out->lux_raw      = lux_raw_local;
    out->lux_filtered = lux_filtered;
    out->lux_est      = lux_est;
    out->moisture_mv  = moisture_mv;

    LOG_INF("Temp. = %d.%02d C, RH = %u.%02u %%, "
            "LUX(raw|filter|est.) = (%u|%u|%ld) lux, Moisture = %d mV",
            temp_c_x100 / 100,
            (temp_c_x100 >= 0 ? temp_c_x100 % 100 : -(temp_c_x100 % 100)),
            rh_x100 / 100,
            rh_x100 % 100,
            (unsigned int)lux_raw_local,
            (unsigned int)lux_filtered,
            (long)lux_est,
            moisture_mv);

    /* --------- Plant profile comparisons (optional) ----------------------- */

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

        /* Light */
        if (lux_est < profile->lux_min) {
            LOG_WRN("Plant profile: light too LOW (min=%d)",
                    profile->lux_min);
        } else if (lux_est > profile->lux_max) {
            LOG_WRN("Plant profile: light too HIGH (max=%d)",
                    profile->lux_max);
        }

        /* Soil moisture */
        if (moisture_mv < profile->moisture_min_mv) {
            LOG_WRN("Plant profile: soil too dry (min=%d mV)",
                    profile->moisture_min_mv);
        } else if (moisture_mv > profile->moisture_max_mv) {
            LOG_WRN("Plant profile: soil too wet (max=%d mV)",
                    profile->moisture_max_mv);
        }
    }

    return 0;
}
