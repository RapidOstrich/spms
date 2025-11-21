#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* Bluetooth dependencies */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

/* GPIO for connection LED */
#include <zephyr/drivers/gpio.h>

/* Local modules */
#include "log_store.h"   /* struct plant_profile, log_iter_*, plant_profile_save() */
#include "spms_ble.h"

LOG_MODULE_REGISTER(SPMS_BLE, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* External plant profile (defined in main.c)                                */
/* ------------------------------------------------------------------------- */

extern struct plant_profile g_profile;

/* ------------------------------------------------------------------------- */
/* Connection LED (optional, configured by main)                             */
/* ------------------------------------------------------------------------- */

static const struct gpio_dt_spec *conn_led_spec;

/* ------------------------------------------------------------------------- */
/* Backing storage for GATT characteristics                                  */
/* ------------------------------------------------------------------------- */

static uint8_t  debug_value = 0x42;
static bool     debug_notify_enabled;

static bool     temp_notify_enabled;
static bool     rh_notify_enabled;

/* Latest sensor values (cached for GATT reads/notifications) */
static int16_t  last_temp_c_x100;
static uint16_t last_rh_x100;

/* ------------------------------------------------------------------------- */
/* Log dump state                                                            */
/* ------------------------------------------------------------------------- */

static bool            log_data_notify_enabled;
static bool            log_dump_active;
static struct log_iter log_dump_it;
static struct log_record log_dump_rec;

/* How many records to send per work-cycle; tune as needed */
#define LOG_DUMP_RECORDS_PER_CYCLE  4

static void log_dump_work_handler(struct k_work *work);

/* Work item used to pump out notifications asynchronously */
K_WORK_DELAYABLE_DEFINE(log_dump_work, log_dump_work_handler);

/* ------------------------------------------------------------------------- */
/* Advertising data / UUIDs / GATT services                                  */
/* ------------------------------------------------------------------------- */

#define DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN  (sizeof(DEVICE_NAME) - 1)

/* Advertising payload: flags */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

/* Scan response: complete device name */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Main SPMS service UUIDs:
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

/* Plant profile service UUIDs:
 *
 * Service:  12345679-1234-5678-1234-56789abcdef0
 * Profile:  12345679-1234-5678-1234-56789abcdef1
 */

#define BT_UUID_SPMS_PLANT_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x56789abcdef0ULL)

#define BT_UUID_SPMS_PLANT_PROFILE_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x1234, 0x56789abcdef1ULL)

static struct bt_uuid_128 spms_plant_service_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_PLANT_SERVICE_VAL);
static struct bt_uuid_128 spms_plant_profile_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_PLANT_PROFILE_CHAR_VAL);

/* Log dump service UUIDs:
 *
 * Service:  1234567A-1234-5678-1234-56789abcdef0
 * Ctrl chr: 1234567A-1234-5678-1234-56789abcdef1
 * Data chr: 1234567A-1234-5678-1234-56789abcdef2
 */

#define BT_UUID_SPMS_LOG_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x1234567A, 0x1234, 0x5678, 0x1234, 0x56789abcdef0ULL)

#define BT_UUID_SPMS_LOG_CTRL_CHAR_VAL \
    BT_UUID_128_ENCODE(0x1234567A, 0x1234, 0x5678, 0x1234, 0x56789abcdef1ULL)

#define BT_UUID_SPMS_LOG_DATA_CHAR_VAL \
    BT_UUID_128_ENCODE(0x1234567A, 0x1234, 0x5678, 0x1234, 0x56789abcdef2ULL)

static struct bt_uuid_128 spms_log_service_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_LOG_SERVICE_VAL);
static struct bt_uuid_128 spms_log_ctrl_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_LOG_CTRL_CHAR_VAL);
static struct bt_uuid_128 spms_log_data_uuid = BT_UUID_INIT_128(
    BT_UUID_SPMS_LOG_DATA_CHAR_VAL);

/* ------------------------------------------------------------------------- */
/* Generic GATT read helpers                                                 */
/* ------------------------------------------------------------------------- */

static ssize_t read_u8(struct bt_conn *conn,
                       const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len,
                       uint16_t offset)
{
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             value, sizeof(*value));
}

static ssize_t read_s16(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len,
                        uint16_t offset)
{
    const int16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             value, sizeof(*value));
}

static ssize_t read_u16(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len,
                        uint16_t offset)
{
    const uint16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             value, sizeof(*value));
}

/* ------------------------------------------------------------------------- */
/* Plant profile GATT handlers                                               */
/* ------------------------------------------------------------------------- */

static ssize_t plant_profile_read(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len,
                                  uint16_t offset)
{
    const struct plant_profile *p = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             p, sizeof(*p));
}

static ssize_t plant_profile_write(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags)
{
    struct plant_profile *p = attr->user_data;

    if (offset + len > sizeof(*p)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(((uint8_t *)p) + offset, buf, len);

    LOG_INF("Plant profile written:");
    LOG_INF("  Temp:   %d..%d (0.01 C)",
            p->temp_min_x100, p->temp_max_x100);
    LOG_INF("  RH:     %d..%d (0.01 %%RH)",
            p->rh_min_x100, p->rh_max_x100);
    LOG_INF("  Moist:  %d..%d mV",
            p->moisture_min_mv, p->moisture_max_mv);
    LOG_INF("  Lux:    %d..%d",
            p->lux_min, p->lux_max);

    int rc = plant_profile_save(p);
    if (rc) {
        LOG_ERR("Failed to save plant profile to NVS (rc=%d)", rc);
        /* Still accept the write from the client's perspective. */
    }

    return len;
}

/* ------------------------------------------------------------------------- */
/* Debug characteristic write                                                */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* CCC callbacks                                                             */
/* ------------------------------------------------------------------------- */

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

static void log_data_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value)
{
    log_data_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Log data notifications %s",
            log_data_notify_enabled ? "enabled" : "disabled");

    if (!log_data_notify_enabled) {
        /* If notifications are turned off, stop any ongoing dump */
        log_dump_active = false;
    }
}

/* ------------------------------------------------------------------------- */
/* Log Control characteristic                                                */
/* ------------------------------------------------------------------------- */
/* Log Control commands:
 *   0x00 = stop/abort any ongoing dump
 *   0x01 = start full dump from oldest to newest
 */
static ssize_t log_ctrl_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags)
{
    const uint8_t *data = buf;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t cmd = data[0];

    switch (cmd) {
    case 0x00: /* stop */
        LOG_INF("Log dump: stop requested");
        log_dump_active = false;
        break;

    case 0x01: /* start full dump */
    {
        int rc = log_iter_begin(&log_dump_it);
        if (rc == -ENOENT) {
            LOG_INF("Log dump: no records to dump");
            log_dump_active = false;
        } else if (rc) {
            LOG_ERR("log_iter_begin failed (rc=%d)", rc);
            log_dump_active = false;
        } else {
            LOG_INF("Log dump: starting from oldest record");
            log_dump_active = true;

            /* Kick the work item immediately */
            k_work_schedule(&log_dump_work, K_NO_WAIT);
        }
        break;
    }

    default:
        LOG_WRN("Log ctrl: unknown command 0x%02x", cmd);
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
    }

    return len;
}

/* ------------------------------------------------------------------------- */
/* GATT service definitions                                                  */
/* ------------------------------------------------------------------------- */
/* Attribute indices (for reference):
 *   spms_svc:
 *     0: Primary Service
 *
 *     1: Debug Characteristic Declaration
 *     2: Debug Characteristic Value
 *     3: Debug CUD
 *     4: Debug CCC
 *
 *     5: Temp Characteristic Declaration
 *     6: Temp Characteristic Value
 *     7: Temp CUD
 *     8: Temp CCC
 *
 *     9:  RH Characteristic Declaration
 *    10:  RH Characteristic Value
 *    11:  RH CUD
 *    12:  RH CCC
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
                           read_s16, NULL, &last_temp_c_x100),
    BT_GATT_CUD("Temperature (0.01 C)", BT_GATT_PERM_READ),
    BT_GATT_CCC(temp_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Humidity: read / notify (0.01 %RH) */
    BT_GATT_CHARACTERISTIC(&spms_rh_uuid.uuid,
                           BT_GATT_CHRC_READ |
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_u16, NULL, &last_rh_x100),
    BT_GATT_CUD("Humidity (0.01 %RH)", BT_GATT_PERM_READ),
    BT_GATT_CCC(rh_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Plant Profile GATT Service */

BT_GATT_SERVICE_DEFINE(spms_plant_svc,
    BT_GATT_PRIMARY_SERVICE(&spms_plant_service_uuid),

    /* Single characteristic: full plant_profile struct as blob */
    BT_GATT_CHARACTERISTIC(&spms_plant_profile_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           plant_profile_read,
                           plant_profile_write,
                           &g_profile),

    BT_GATT_CUD("Plant Profile", BT_GATT_PERM_READ)
);

/* Log Dump GATT Service
 *
 * Attribute layout (for reference):
 *   0: Primary Service
 *
 *   1: Log Ctrl Characteristic Declaration
 *   2: Log Ctrl Characteristic Value
 *   3: Log Ctrl CUD
 *
 *   4: Log Data Characteristic Declaration
 *   5: Log Data Characteristic Value
 *   6: Log Data CUD
 *   7: Log Data CCC
 */
#define SPMS_LOG_DATA_VAL_ATTR_INDEX  5

BT_GATT_SERVICE_DEFINE(spms_log_svc,
    BT_GATT_PRIMARY_SERVICE(&spms_log_service_uuid),

    /* Log Control characteristic: write-only */
    BT_GATT_CHARACTERISTIC(&spms_log_ctrl_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL,            /* no read */
                           log_ctrl_write,  /* write handler */
                           NULL),
    BT_GATT_CUD("Log Control", BT_GATT_PERM_READ),

    /* Log Data characteristic: notify-only (20-byte struct log_record) */
    BT_GATT_CHARACTERISTIC(&spms_log_data_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL,    /* no read; use notifications */
                           NULL,
                           NULL),
    BT_GATT_CUD("Log Data", BT_GATT_PERM_READ),
    BT_GATT_CCC(log_data_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ------------------------------------------------------------------------- */
/* Log dump work handler                                                     */
/* ------------------------------------------------------------------------- */

static void log_dump_work_handler(struct k_work *work)
{
    int rc;
    int sent = 0;

    ARG_UNUSED(work);

    if (!log_dump_active || !log_data_notify_enabled) {
        return;
    }

    while (sent < LOG_DUMP_RECORDS_PER_CYCLE) {
        uint16_t key = 0;

        rc = log_iter_next(&log_dump_it, &log_dump_rec, &key);
        if (rc == -ENOENT) {
            LOG_INF("Log dump: finished streaming records");
            log_dump_active = false;
            return;
        } else if (rc) {
            LOG_ERR("log_iter_next failed (rc=%d), aborting dump", rc);
            log_dump_active = false;
            return;
        }

        rc = bt_gatt_notify(NULL,
                            &spms_log_svc.attrs[SPMS_LOG_DATA_VAL_ATTR_INDEX],
                            &log_dump_rec,
                            sizeof(log_dump_rec));
        if (rc) {
            LOG_WRN("bt_gatt_notify(log) failed (rc=%d), aborting dump", rc);
            log_dump_active = false;
            return;
        }

        sent++;
    }

    /* Schedule the next batch a bit later to avoid hogging the link */
    k_work_schedule(&log_dump_work, K_MSEC(50));
}

/* ------------------------------------------------------------------------- */
/* Connection callbacks                                                      */
/* ------------------------------------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        return;
    }

    LOG_INF("Connected");

    if (conn_led_spec) {
        gpio_pin_set_dt(conn_led_spec, 1);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);

    if (conn_led_spec) {
        gpio_pin_set_dt(conn_led_spec, 0);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

int spms_ble_init(const struct gpio_dt_spec *conn_led)
{
    int err;

    conn_led_spec = conn_led;  /* may be NULL */

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    LOG_INF("Bluetooth initialized");

    err = bt_le_adv_start(BT_LE_ADV_CONN,
                          ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return err;
    }

    LOG_INF("Advertising started (connectable, name = \"%s\")", DEVICE_NAME);

    return 0;
}

void spms_ble_tick_1s(int16_t temp_c_x100, uint16_t rh_x100)
{
    /* Update cached sensor values backing the GATT attributes */
    last_temp_c_x100 = temp_c_x100;
    last_rh_x100     = rh_x100;

    /* Demo counter for debug characteristic */
    debug_value++;

    if (debug_notify_enabled) {
        /* Debug value at attr index 2 */
        bt_gatt_notify(NULL, &spms_svc.attrs[2],
                       &debug_value, sizeof(debug_value));
    }

    if (temp_notify_enabled) {
        /* Temp value at attr index 6 */
        bt_gatt_notify(NULL, &spms_svc.attrs[6],
                       &last_temp_c_x100, sizeof(last_temp_c_x100));
    }

    if (rh_notify_enabled) {
        /* RH value at attr index 10 */
        bt_gatt_notify(NULL, &spms_svc.attrs[10],
                       &last_rh_x100, sizeof(last_rh_x100));
    }
}
