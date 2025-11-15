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
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* ------------------------------------------------------------------------- */
/* Bluetooth Dependencies                                                    */
/* ------------------------------------------------------------------------- */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

/* ------------------------------------------------------------------------- */
/* Local Includes                                                            */
/* ------------------------------------------------------------------------- */
#include "sen0114.h"
#include "sen0390.h"
#include "sen0546.h"
#include "log_store.h"

#define DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN  (sizeof(DEVICE_NAME) - 1)

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

/* Latest converted values (0.01 units) */
static int16_t  temp_c_x100 = 0;  /* e.g. 2534 = 25.34 °C */
static uint16_t rh_x100     = 0;  /* e.g. 5034 = 50.34 %RH */

/* Latest raw sensor words (for logging) */
static uint32_t lux_raw       = 0;
static uint16_t last_temp_raw = 0;
static uint16_t last_rh_raw   = 0;

/* Calibration constants – tweak these empirically */
static const int32_t L_A = 0;        // lux in dark
static const int32_t L_B = 20000;    // lux in bright env you used as reference

static const uint32_t R_A = 100663296;
static const uint32_t R_B = 4155179520;

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
/* Advertising data                                                          */
/* ------------------------------------------------------------------------- */

/* Advertising payload:
 *   - Flags: General Discoverable, BR/EDR not supported
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

/* Scan response payload:
 *   - Complete device name ("SPMS")
 */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* ------------------------------------------------------------------------- */
/* Custom GATT Service & Characteristics                                     */
/* ------------------------------------------------------------------------- */
/* UUIDs (example values):
 *
 * Service:   12345678-1234-5678-1234-56789abcdef0
 * Debug chr: 12345678-1234-5678-1234-56789abcdef1
 * Temp chr:  12345678-1234-5678-1234-56789abcdef2
 * RH chr:    12345678-1234-5678-1234-56789abcdef3
 */

#define BT_UUID_SPMS_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0ULL)

#define BT_UUID_SPMS_DEBUG_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1ULL)

#define BT_UUID_SPMS_TEMP_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2ULL)

#define BT_UUID_SPMS_RH_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3ULL)

static struct bt_uuid_128 spms_service_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_SERVICE_VAL);
static struct bt_uuid_128 spms_debug_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_DEBUG_CHAR_VAL);
static struct bt_uuid_128 spms_temp_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_TEMP_CHAR_VAL);
static struct bt_uuid_128 spms_rh_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_RH_CHAR_VAL);

/* Backing storage */
static uint8_t  debug_value = 0x42;
static bool     debug_notify_enabled;

static bool     temp_notify_enabled;
static bool     rh_notify_enabled;

/* --- Generic read helpers ------------------------------------------------ */

static ssize_t read_u8(struct bt_conn *conn,
                       const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    const uint8_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             value, sizeof(*value));
}

static ssize_t read_s16(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             value, sizeof(*value));
}

static ssize_t read_u16(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const uint16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             value, sizeof(*value));
}

/* --- Debug characteristic write ----------------------------------------- */

static ssize_t debug_write(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           const void *buf, uint16_t len,
                           uint16_t offset, uint8_t flags)
{
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    LOG_INF("Debug char written: 0x%02x", *value);

    return len;
}

/* --- CCC callbacks ------------------------------------------------------ */

static void debug_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    debug_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Debug notifications %s",
            debug_notify_enabled ? "enabled" : "disabled");
}

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    temp_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Temp notifications %s",
            temp_notify_enabled ? "enabled" : "disabled");
}

static void rh_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    rh_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("RH notifications %s",
            rh_notify_enabled ? "enabled" : "disabled");
}

/* --- Service definition -------------------------------------------------- */
/* Attribute indices:
 *   0: Primary Service
 *
 *   1: Debug Characteristic Declaration
 *   2: Debug Characteristic Value
 *   3: Debug CUD
 *   4: Debug CCC
 *
 *   5: Temp Characteristic Declaration
 *   6: Temp Characteristic Value
 *   7: Temp CUD
 *   8: Temp CCC
 *
 *   9: RH Characteristic Declaration
 *  10: RH Characteristic Value
 *  11: RH CUD
 *  12: RH CCC
 */
BT_GATT_SERVICE_DEFINE(spms_svc,
    BT_GATT_PRIMARY_SERVICE(&spms_service_uuid),

    /* Debug: read / write / notify */
    BT_GATT_CHARACTERISTIC(&spms_debug_uuid.uuid,
                           BT_GATT_CHRC_READ |
                           BT_GATT_CHRC_WRITE |
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ |
                           BT_GATT_PERM_WRITE,
                           read_u8, debug_write, &debug_value),
    BT_GATT_CUD("Debug Value", BT_GATT_PERM_READ),
    BT_GATT_CCC(debug_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Temperature: read / notify (0.01 °C) */
    BT_GATT_CHARACTERISTIC(&spms_temp_uuid.uuid,
                           BT_GATT_CHRC_READ |
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_s16, NULL, &temp_c_x100),
    BT_GATT_CUD("Temperature (0.01 C)", BT_GATT_PERM_READ),
    BT_GATT_CCC(temp_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Humidity: read / notify (0.01 %RH) */
    BT_GATT_CHARACTERISTIC(&spms_rh_uuid.uuid,
                           BT_GATT_CHRC_READ |
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_u16, NULL, &rh_x100),
    BT_GATT_CUD("Humidity (0.01 %RH)", BT_GATT_PERM_READ),
    BT_GATT_CCC(rh_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ------------------------------------------------------------------------- */
/* Connection callbacks                                                      */
/* ------------------------------------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
        /* LED1 ON when connected */
        gpio_pin_set_dt(&led_conn, 1);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    /* LED1 OFF when no connection */
    gpio_pin_set_dt(&led_conn, 0);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ------------------------------------------------------------------------- */
/* Sensor read helper                                                        */
/* ------------------------------------------------------------------------- */

int32_t sen0390_raw_to_lux(uint32_t lux_raw)
{
    // Guard against division by zero / bogus calibration
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

static int update_sensor_values(void)
{
    uint32_t lux_raw_local  = 0;
    uint16_t temp_raw = 0;
    uint16_t rh_raw   = 0;
    int ret;

    ret = query_sen0390(&sen0390_i2c, &lux_raw_local);
    if (ret < 0) {
        LOG_ERR("query_sen0390 failed: %d", ret);
        return ret;
    }

    ret = query_sen0546(&sen0546_i2c, &temp_raw, &rh_raw);
    if (ret < 0) {
        LOG_ERR("query_sen0546 failed: %d", ret);
        return ret;
    }

    last_temp_raw = temp_raw;
    last_rh_raw   = rh_raw;
    // optional:
    // last_lux_raw = lux_raw_local;
    lux_raw = lux_raw_local;

    float temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    float rh     = 100.0f * ((float)rh_raw / 65535.0f);

    temp_c_x100 = (int16_t)(temp_c * 100.0f);
    rh_x100     = (uint16_t)(rh * 100.0f);

    int32_t lux_est = sen0390_raw_to_lux(lux_raw_local);

    LOG_INF("Sensor: T = %d.%02d °C, RH = %u.%02u %% LUX(raw) = %u est = %ld lux",
           temp_c_x100 / 100,
           (temp_c_x100 >= 0 ? temp_c_x100 % 100 : -(temp_c_x100 % 100)),
           rh_x100 / 100,
           rh_x100 % 100,
           (unsigned int)lux_raw_local,
           (long)lux_est
    );

    // Here is where you'd compare against the plant profile
    // if (lux_est < profile.min_lux) { ... }
    // if (lux_est > profile.max_lux) { ... }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Logging helper (once per minute) + LED2 flash                             */
/* ------------------------------------------------------------------------- */

static int log_current_record(void)
{
    struct log_record rec;
    uint16_t key = 0;

    /* Timestamp in seconds (monotonic is fine for this use case) */
    rec.ts_s       = (uint32_t)(k_uptime_get() / 1000U);
    rec.t_raw      = temp_c_x100;
    rec.rh_raw     = rh_x100;
    rec.lux_raw    = lux_raw;
    rec.moisture_mv = 0; /* will be filled when moisture integration is done */

    int rc = log_store_append(&rec, &key);
    if (rc == 0) {
        LOG_INF("Log: stored record at key=0x%04x ts=%u t_raw=%d rh_raw=%d lux_raw=%u moisture_mv=%d",
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

    if (!device_is_ready(sen0390_i2c.bus)) {
        LOG_ERR("I2C bus %s not ready!", sen0390_i2c.bus->name);
        return 0;
    }
    if (!device_is_ready(sen0546_i2c.bus)) {
        LOG_ERR("I2C bus %s not ready!", sen0546_i2c.bus->name);
        return 0;
    }
    LOG_INF("I2C ready, addr=0x%02x", sen0546_i2c.addr);

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

    /* Initialize the Bluetooth stack */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    LOG_INF("Bluetooth initialized");

    /* Start connectable advertising.
     * BT_LE_ADV_CONN is deprecated in newer APIs but fine in this SDK.
     */
    err = bt_le_adv_start(BT_LE_ADV_CONN,
                          ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return 0;
    }

    LOG_INF("Advertising started (connectable, name = \"%s\")", DEVICE_NAME);

    int seconds_since_sensor = 0;
    int seconds_since_log    = 0;

    while (1) {
        k_sleep(K_SECONDS(1));

        /* Demo counter for debug characteristic */
        debug_value++;

        if (debug_notify_enabled) {
            /* Debug value at attr index 2 */
            bt_gatt_notify(NULL, &spms_svc.attrs[2],
                           &debug_value, sizeof(debug_value));
        }

        /* Periodic timing */
        seconds_since_sensor++;
        seconds_since_log++;

        /* Every 5 seconds: read sensor + print */
        if (seconds_since_sensor >= 5) {
            seconds_since_sensor = 0;
            update_sensor_values();
        }

        /* Every 60 seconds: log current sample to NVS */
        if (seconds_since_log >= 60) {
            seconds_since_log = 0;
            log_current_record();
        }

        /* Notify latest sensor values if client subscribed */
        if (temp_notify_enabled) {
            /* Temp value at attr index 6 */
            bt_gatt_notify(NULL, &spms_svc.attrs[6],
                           &temp_c_x100, sizeof(temp_c_x100));
        }

        if (rh_notify_enabled) {
            /* RH value at attr index 10 */
            bt_gatt_notify(NULL, &spms_svc.attrs[10],
                           &rh_x100, sizeof(rh_x100));
        }
    }

    return 0;
}
