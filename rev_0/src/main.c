#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* ------------------------------------------------------------------------- */
/* Logging module                                                            */
/* ------------------------------------------------------------------------- */

LOG_MODULE_REGISTER(spms_main, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* Board Config Dependencies                                                 */
/* ------------------------------------------------------------------------- */
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/* ------------------------------------------------------------------------- */
/* Local Includes                                                            */
/* ------------------------------------------------------------------------- */
#include "sen0114.h"
#include "sen0390.h"
#include "sen0546.h"
#include "log_store.h"
#include "spms_ble.h"
#include "spms_sensors.h"

/* ------------------------------------------------------------------------- */
/* Plant profile (loaded from NVS)                                           */
/* ------------------------------------------------------------------------- */

/* Non-static: referenced from spms_ble.c via extern */
struct plant_profile g_profile;

/* Latest sensor snapshot (for logging + BLE) */
static struct spms_sensor_snapshot g_sensors;

/* ------------------------------------------------------------------------- */
/* LEDs                                                                      */
/* ------------------------------------------------------------------------- */
/* LED1 (board marking) -> led0 alias: connection indicator
 * LED2 (board marking) -> led1 alias: log flash
 */

#define LED_CONN_NODE DT_ALIAS(led0)
#define LED_LOG_NODE  DT_ALIAS(led1)

#if !DT_NODE_HAS_STATUS(LED_CONN_NODE, okay)
#error "LED connection alias (led0) not found in devicetree"
#endif

#if !DT_NODE_HAS_STATUS(LED_LOG_NODE, okay)
#error "LED log alias (led1) not found in devicetree"
#endif

static const struct gpio_dt_spec led_conn = GPIO_DT_SPEC_GET(LED_CONN_NODE, gpios);
static const struct gpio_dt_spec led_log  = GPIO_DT_SPEC_GET(LED_LOG_NODE, gpios);

/* ------------------------------------------------------------------------- */
/* Logging helper (once per minute) + LED2 flash                             */
/* ------------------------------------------------------------------------- */

static int log_current_record(void)
{
    struct log_record rec;
    uint16_t key = 0;

    /* Timestamp in seconds (monotonic is fine for this use case) */
    rec.ts_s        = (uint32_t)(k_uptime_get() / 1000U);
    rec.t_raw       = g_sensors.temp_c_x100;
    rec.rh_raw      = g_sensors.rh_x100;
    rec.lux_raw     = g_sensors.lux_raw;
    rec.moisture_mv = 0; /* will be filled when moisture integration is done */

    int rc = log_store_append(&rec, &key);
    if (rc == 0) {
        LOG_INF("Log: stored record at key=0x%04x ts=%u t_raw=%d rh_raw=%d "
                "lux_raw=%u moisture_mv=%d",
                key, rec.ts_s, rec.t_raw, rec.rh_raw,
                (unsigned int)rec.lux_raw, rec.moisture_mv);

        /* Flash LED2 briefly to indicate a log event */
        gpio_pin_set_dt(&led_log, 1);
        k_sleep(K_MSEC(100));
        gpio_pin_set_dt(&led_log, 0);
    } else {
        LOG_ERR("Log: append failed rc=%d", rc);
    }

    return rc;
}

/* ------------------------------------------------------------------------- */
/* Main                                                                      */
/* ------------------------------------------------------------------------- */

int main(void)
{
    int err;

    LOG_INF("SPMS_REV_0");

    /* Initialize LEDs */
    if (!device_is_ready(led_conn.port)) {
        LOG_ERR("LED connection port not ready!");
        return 0;
    }
    if (!device_is_ready(led_log.port)) {
        LOG_ERR("LED log port not ready!");
        return 0;
    }

    gpio_pin_configure_dt(&led_conn, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_log, GPIO_OUTPUT_INACTIVE);

    /* Initialize sensor subsystem (I2C + basic checks) */
    err = spms_sensors_init();
    if (err) {
        LOG_ERR("spms_sensors_init failed (err %d)", err);
        /* Optionally: return 0; for now, continue to allow BLE/logging startup */
    }

    /* Initialize log_store (NVS-backed flash storage) */
    err = log_store_init();
    if (err) {
        LOG_ERR("log_store_init failed (err %d)", err);
        /* Keep running BLE/sensor even if logging is broken */
    } else {
        size_t cap = log_store_capacity_records(sizeof(struct log_record));
        LOG_INF("log_store initialized, approx capacity ~%u records",
                (unsigned int)cap);
    }

    /* Load or initialize plant profile in NVS */
    err = plant_profile_load_or_init(&g_profile);
    if (err) {
        LOG_ERR("plant_profile_load_or_init failed (err %d)", err);
    }

    /* Initialize Bluetooth stack + GATT services + advertising */
    err = spms_ble_init(&led_conn);
    if (err) {
        LOG_ERR("spms_ble_init failed (err %d)", err);
        /* Optionally: return 0; and run without BLE */
    }

    int seconds_since_sensor = 0;
    int seconds_since_log    = 0;

    while (1) {
        k_sleep(K_SECONDS(1));

        /* Periodic timing */
        seconds_since_sensor++;
        seconds_since_log++;

        /* Every 5 seconds: read sensors + compare against profile */
        if (seconds_since_sensor >= 5) {
            seconds_since_sensor = 0;

            err = spms_sensors_update(&g_profile, &g_sensors);
            if (err) {
                LOG_ERR("spms_sensors_update failed (err %d)", err);
            }
        }

        /* Every 60 seconds: log current sample to NVS */
        if (seconds_since_log >= 60) {
            seconds_since_log = 0;
            log_current_record();
        }

        /* 1 Hz BLE maintenance + notifications using latest sensor values */
        spms_ble_tick_1s(g_sensors.temp_c_x100, g_sensors.rh_x100);
    }

    return 0;
}
