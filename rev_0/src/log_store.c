/**
 * @file
 * @brief NVS-backed log storage and plant profile persistence.
 *
 * This module:
 *   - Uses Zephyr's NVS on the "storage" partition to store sequential
 *     log_record entries in a ring buffer.
 *   - Persists a small plant_profile struct for threshold settings.
 *   - Provides an iterator for walking records from oldest to newest.
 *   - Offers helper functions to dump records for debugging.
 */

#include <zephyr/kernel.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <zephyr/logging/log.h>

#include "log_store.h"

LOG_MODULE_REGISTER(LOG_STORE, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

/* nRF52 internal flash erase page is 4096 bytes. Keep NVS sector aligned. */
#define NVS_SECTOR_SIZE_DEFAULT   4096U

/*
 * Sequencing:
 *   - Latest sequence number is stored at KEY_SEQ.
 *   - Each record is written at key = seq (0x0100..0x7FFF).
 *   - On overflow, we wrap to 0x0100 and set a wrap flag in NVS.
 */
#define KEY_SEQ            0x0001
#define KEY_WRAP_FLAG      0x0002
#define KEY_MIN_DATA       0x0100
#define KEY_MAX_DATA       0x7FFF

/* Single plant profile stored as a blob. */
#define KEY_PLANT_PROFILE  0x0003

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

struct log_store {
    struct nvs_fs nvs;
    struct k_mutex lock;
    uint16_t last_seq;     /* Latest sequence persisted (0 means none yet). */
    bool     wrapped;      /* True if we have wrapped at least once. */

    size_t part_size;
    size_t sector_size;
    size_t sector_count;
};

static struct log_store g_ls;

/* ------------------------------------------------------------------------- */
/* Internal helpers                                                          */
/* ------------------------------------------------------------------------- */

/* Persist/restore small values (u16) in NVS. */
static int persist_u16(struct nvs_fs *n, uint16_t key, uint16_t v)
{
    return nvs_write(n, key, &v, sizeof(v));
}

static int read_u16(struct nvs_fs *n, uint16_t key, uint16_t *out)
{
    int rc = nvs_read(n, key, out, sizeof(*out));
    if (rc == sizeof(*out)) {
        return 0;
    }
    return (rc < 0) ? rc : -ENOENT;
}

/* Compute a data key from a sequence value, wrapping into the data range. */
static uint16_t seq_to_key(uint16_t seq)
{
    if (seq < KEY_MIN_DATA) {
        return KEY_MIN_DATA + (seq % (KEY_MAX_DATA - KEY_MIN_DATA + 1));
    }

    if (seq > KEY_MAX_DATA) {
        uint32_t span = (KEY_MAX_DATA - KEY_MIN_DATA + 1);
        return KEY_MIN_DATA + ((seq - KEY_MIN_DATA) % span);
    }

    return seq;
}

struct log_store *log_store_get(void)
{
    return &g_ls;
}

/* Advance to the next sequence/key. Must be called with ls->lock held. */
static int advance_sequence_locked(struct log_store *ls,
                                   uint16_t *out_seq,
                                   uint16_t *out_key)
{
    /* Next sequence; handle wrap back into data range. */
    uint32_t next = (ls->last_seq == 0)
                    ? (KEY_MIN_DATA)
                    : (ls->last_seq + 1);

    if (next > KEY_MAX_DATA) {
        next = KEY_MIN_DATA;
        ls->wrapped = true;
        (void)persist_u16(&ls->nvs, KEY_WRAP_FLAG, 1);
    }

    uint16_t key = seq_to_key((uint16_t)next);

    /* Persist the new last sequence. */
    int rc = persist_u16(&ls->nvs, KEY_SEQ, (uint16_t)next);
    if (rc < 0) {
        return rc;
    }

    ls->last_seq = (uint16_t)next;

    if (out_seq) {
        *out_seq = (uint16_t)next;
    }
    if (out_key) {
        *out_key = key;
    }

    return 0;
}

/* Iterator helpers. */
static uint16_t iter_first_key(struct log_store *ls)
{
    if (!ls->last_seq) {
        return 0;
    }

    if (!ls->wrapped) {
        /* No wrap: first record is at KEY_MIN_DATA. */
        return KEY_MIN_DATA;
    } else {
        /*
         * Wrapped: oldest record is one past last_seq
         * (wrapped back into range if needed).
         */
        uint16_t first_seq = ls->last_seq + 1;
        if (first_seq > KEY_MAX_DATA) {
            first_seq = KEY_MIN_DATA;
        }
        return seq_to_key(first_seq);
    }
}

static uint16_t key_advance(uint16_t k)
{
    return (k >= KEY_MAX_DATA) ? KEY_MIN_DATA : (uint16_t)(k + 1);
}

/* ------------------------------------------------------------------------- */
/* Initialization                                                            */
/* ------------------------------------------------------------------------- */

int log_store_init(void)
{
    struct log_store *ls = &g_ls;
    int rc;

    k_mutex_init(&ls->lock);

    const struct device *flash_dev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    if (!device_is_ready(flash_dev)) {
        return -ENODEV;
    }

    const struct flash_area *fa;
    rc = flash_area_open(FIXED_PARTITION_ID(storage), &fa);
    if (rc) {
        return rc;
    }

    ls->part_size    = fa->fa_size;
    ls->sector_size  = NVS_SECTOR_SIZE_DEFAULT;
    ls->sector_count = ls->part_size / ls->sector_size;

    ls->nvs.flash_device = flash_dev;
    ls->nvs.offset       = fa->fa_off;
    ls->nvs.sector_size  = ls->sector_size;
    ls->nvs.sector_count = ls->sector_count;

    rc = nvs_mount(&ls->nvs);
    flash_area_close(fa);
    if (rc) {
        return rc;
    }

    /* Restore persisted sequence/wrap state (if they exist). */
    uint16_t v = 0;
    rc = read_u16(&ls->nvs, KEY_SEQ, &v);
    ls->last_seq = (rc == 0) ? v : 0;

    uint16_t w = 0;
    rc = read_u16(&ls->nvs, KEY_WRAP_FLAG, &w);
    ls->wrapped = (rc == 0 && w == 1);

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Core log storage API                                                      */
/* ------------------------------------------------------------------------- */

int log_store_append(const struct log_record *rec, uint16_t *out_key)
{
    if (!rec) {
        return -EINVAL;
    }

    struct log_store *ls = &g_ls;
    int rc;
    uint16_t key;

    k_mutex_lock(&ls->lock, K_FOREVER);
    rc = advance_sequence_locked(ls, NULL, &key);

    if (rc == 0) {
        rc = nvs_write(&ls->nvs, key, rec, sizeof(*rec));
        if (rc == (int)sizeof(*rec)) {
            rc = 0;
        }
    }

    k_mutex_unlock(&ls->lock);

    if (rc == 0 && out_key) {
        *out_key = key;
    }

    return rc;
}

int log_store_read_latest(struct log_record *out, uint16_t *out_key)
{
    struct log_store *ls = &g_ls;
    if (ls->last_seq == 0) {
        return -ENOENT;
    }

    uint16_t key = seq_to_key(ls->last_seq);
    int rc = nvs_read(&ls->nvs, key, out, sizeof(*out));
    if (rc == (int)sizeof(*out)) {
        if (out_key) {
            *out_key = key;
        }
        return 0;
    }

    return (rc < 0) ? rc : -ENOENT;
}

int log_store_read_key(uint16_t key, struct log_record *out)
{
    struct log_store *ls = &g_ls;
    int rc = nvs_read(&ls->nvs, key, out, sizeof(*out));
    return (rc == (int)sizeof(*out)) ? 0 : ((rc < 0) ? rc : -ENOENT);
}

/* ------------------------------------------------------------------------- */
/* Iteration over stored records                                             */
/* ------------------------------------------------------------------------- */

int log_iter_begin(struct log_iter *it)
{
    if (!it) {
        return -EINVAL;
    }

    struct log_store *ls = &g_ls;

    if (ls->last_seq == 0) {
        return -ENOENT;
    }

    it->cur     = iter_first_key(ls);
    it->end     = seq_to_key(ls->last_seq);
    it->wrapped = ls->wrapped ? 1 : 0;
    it->started = 0;

    return 0;
}

int log_iter_next(struct log_iter *it,
                  struct log_record *out,
                  uint16_t *out_key)
{
    if (!it || !out) {
        return -EINVAL;
    }

    /* If we've already started and advanced past the end, we're done. */
    if (it->started && it->cur == key_advance(it->end)) {
        return -ENOENT;
    }

    it->started = 1;

    /*
     * Read at current key; it's okay if gaps exist (e.g. power-off windows).
     * We transparently skip empty/unreadable slots.
     */
    struct log_store *ls = &g_ls;

    while (1) {
        int rc           = nvs_read(&ls->nvs, it->cur, out, sizeof(*out));
        uint16_t thisKey = it->cur;

        if (thisKey == key_advance(it->end)) {
            return -ENOENT; /* Reached end after attempting read. */
        }

        it->cur = key_advance(it->cur);

        if (rc == (int)sizeof(*out)) {
            if (out_key) {
                *out_key = thisKey;
            }
            return 0;
        }

        /* If slot empty/unreadable, continue to next key. */
    }
}

/* ------------------------------------------------------------------------- */
/* Formatting / administrative helpers                                       */
/* ------------------------------------------------------------------------- */

int log_store_format(void)
{
    struct log_store *ls = &g_ls;
    int rc;

    k_mutex_lock(&ls->lock, K_FOREVER);
    rc = nvs_clear(&ls->nvs);
    if (rc == 0) {
        ls->last_seq = 0;
        ls->wrapped  = false;
        (void)persist_u16(&ls->nvs, KEY_WRAP_FLAG, 0);
    }
    k_mutex_unlock(&ls->lock);

    return rc;
}

size_t log_store_capacity_records(size_t record_size)
{
    struct log_store *ls = &g_ls;
    if (ls->part_size == 0) {
        return 0;
    }

    /* NVS has metadata and GC overhead; conservatively use ~70% for payload. */
    size_t usable = (ls->part_size * 70) / 100;

    /* Empirical per-record overhead in NVS is small but non-zero; budget ~16 B. */
    size_t per = record_size + 16U;

    return (per == 0) ? 0 : (usable / per);
}

/* ------------------------------------------------------------------------- */
/* Plant profile persistence                                                 */
/* ------------------------------------------------------------------------- */

int plant_profile_save(const struct plant_profile *p)
{
    if (!p) {
        return -EINVAL;
    }

    struct log_store *ls = &g_ls;
    int rc;

    k_mutex_lock(&ls->lock, K_FOREVER);

    rc = nvs_write(&ls->nvs, KEY_PLANT_PROFILE, p, sizeof(*p));
    if (rc == (int)sizeof(*p)) {
        rc = 0;
    } else if (rc >= 0) {
        rc = -EIO;    /* Short write. */
    }

    k_mutex_unlock(&ls->lock);
    return rc;
}

int plant_profile_load(struct plant_profile *out)
{
    if (!out) {
        return -EINVAL;
    }

    struct log_store *ls = &g_ls;

    int rc = nvs_read(&ls->nvs, KEY_PLANT_PROFILE, out, sizeof(*out));
    if (rc == (int)sizeof(*out)) {
        return 0;
    }

    return (rc < 0) ? rc : -ENOENT;
}

/* Weak hook: application can override this to pretty-print records. */
__attribute__((weak))
void log_store_format_record(const struct log_record *rec, uint16_t key)
{
    const uint8_t *p = (const uint8_t *)rec;

    /* Build the hex dump into a local buffer, then log once. */
    char buf[128];
    int pos = 0;

    pos += snprintk(buf + pos, sizeof(buf) - pos,
                    "key=0x%04X data=", key);

    for (size_t i = 0; i < sizeof(*rec) && pos < (int)sizeof(buf); i++) {
        pos += snprintk(buf + pos, sizeof(buf) - pos,
                        "%02X", p[i]);
    }

    LOG_INF("%s", buf);
}

int log_store_dump_to_printf(void)
{
    struct log_iter   it;
    struct log_record rec;
    uint16_t          key;
    int               rc;

    rc = log_iter_begin(&it);
    if (rc) {
        LOG_INF("log_store: no records (rc=%d)", rc);
        return rc;
    }

    int count = 0;
    while ((rc = log_iter_next(&it, &rec, &key)) == 0) {
        log_store_format_record(&rec, key);
        ++count;
    }

    LOG_INF("log_store: %d record(s)", count);
    return (count > 0) ? count : -ENOENT;
}

/* ------------------------------------------------------------------------- */
/* Plant profile load-or-init helper                                         */
/* ------------------------------------------------------------------------- */

int plant_profile_load_or_init(struct plant_profile *out)
{
    if (!out) {
        return -EINVAL;
    }

    int rc = plant_profile_load(out);
    if (rc == 0) {
        LOG_INF("Plant profile loaded from NVS:");
        LOG_INF("  Temp:   %d..%d (0.01 C)",
                out->temp_min_x100, out->temp_max_x100);
        LOG_INF("  RH:     %d..%d (0.01 %%RH)",
                out->rh_min_x100, out->rh_max_x100);
        LOG_INF("  Moist:  %d..%d mV",
                out->moisture_min_mv, out->moisture_max_mv);
        LOG_INF("  Lux:    %d..%d",
                out->lux_min, out->lux_max);
        return 0;
    }

    LOG_WRN("No plant profile in NVS (rc=%d). Initializing defaults.", rc);

    /* Example generic "house plant" defaults. */
    out->temp_min_x100   = 1800;   /* 18.00 °C */
    out->temp_max_x100   = 2800;   /* 28.00 °C */

    out->rh_min_x100     = 4000;   /* 40.00 %RH */
    out->rh_max_x100     = 7000;   /* 70.00 %RH */

    out->moisture_min_mv = 1000;
    out->moisture_max_mv = 3000;

    out->lux_min         = 500;
    out->lux_max         = 30000;

    rc = plant_profile_save(out);
    if (rc == 0) {
        LOG_INF("Default plant profile saved to NVS.");
    } else {
        LOG_ERR("Failed to save default plant profile (rc=%d)", rc);
    }

    return rc;
}
