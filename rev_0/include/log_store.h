#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Compact on-flash record.
 * - Store timestamp (seconds since boot or epoch),
 *   plus 4 sensor readings:
 *      - temperature (x0.01 °C)
 *      - humidity   (x0.01 %RH)
 *      - lux        (raw 32-bit)
 *      - moisture   (mV)
 *
 * Convert to nicer human units when formatting/printing.
 */
struct __attribute__((__packed__)) log_record {
    uint32_t ts_s;        /* Timestamp, seconds */
    int32_t  t_raw;       /* Temp scaled (e.g. x0.01 °C) */
    int32_t  rh_raw;      /* RH scaled   (e.g. x0.01 %RH) */
    uint32_t lux_raw;     /* Raw 32-bit lux value */
    int32_t  moisture_mv; /* Moisture reading in millivolts */
};

/*
 * Iterator for walking records from oldest -> newest.
 * Used by log_iter_begin() / log_iter_next().
 */
struct log_iter {
    uint16_t cur;      /* current key */
    uint16_t end;      /* last (most recent) key */
    uint8_t  wrapped;  /* internal flag: ring has wrapped at least once */
    uint8_t  started;  /* internal flag: iteration has started */
};

/*
 * Initialize NVS-backed storage.
 * - Locates the 'storage' partition via Partition Manager.
 * - Mounts NVS.
 * Returns 0 on success, <0 on error.
 */
int log_store_init(void);

/*
 * Approximate capacity (in number of records) based on the
 * current NVS configuration.
 *
 * You typically call this with sizeof(struct log_record).
 */
size_t log_store_capacity_records(size_t record_size);

/*
 * Append one record to flash.
 *
 * rec      - pointer to record to write
 * out_key  - optional; if non-NULL, receives the NVS key used
 *
 * Returns 0 on success, <0 on error.
 */
int log_store_append(const struct log_record *rec, uint16_t *out_key);

/*
 * Read the latest record (most recent).
 *
 * out      - record buffer
 * out_key  - optional; receives the key it was read from
 *
 * Returns 0 on success, -ENOENT if no records, or other <0 on error.
 */
int log_store_read_latest(struct log_record *out, uint16_t *out_key);

/*
 * Read a specific key into out.
 * Returns 0 on success, -ENOENT if no record at that key, or other <0 on error.
 */
int log_store_read_key(uint16_t key, struct log_record *out);

/*
 * Start an iteration from oldest -> newest.
 * Fills in 'it' for use with log_iter_next().
 *
 * Returns 0 on success, -ENOENT if no records, or other <0 on error.
 */
int log_iter_begin(struct log_iter *it);

/*
 * Get the next record in iteration.
 *
 * Returns:
 *   0        - one record returned in *out (and key in *out_key if non-NULL)
 *  -ENOENT   - no more records
 *   <0       - other error
 */
int log_iter_next(struct log_iter *it, struct log_record *out, uint16_t *out_key);

/*
 * Erase all stored records in NVS and reset sequence state.
 */
int log_store_format(void);

/*
 * Weak hook that you can override to change how dump_to_printf prints.
 */
void log_store_format_record(const struct log_record *rec, uint16_t key);

/*
 * Print all stored records using printf/logging.
 *
 * @return number of records printed, or <0 on error (e.g. -ENOENT if empty).
 */
int log_store_dump_to_printf(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_STORE_H */
