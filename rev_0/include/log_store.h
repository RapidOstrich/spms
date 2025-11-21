#ifndef LOG_STORE_H
#define LOG_STORE_H

/**
 * @file
 * @brief Flash-backed log storage and plant profile persistence.
 *
 * This module provides:
 *   - A compact on-flash record format for periodic sensor logs
 *   - An iterator API to walk log records from oldest to newest
 *   - Helpers to format and dump logs for debugging
 *   - Persistent storage of the plant profile in NVS
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Plant profile                                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief Plant profile thresholds stored in NVS.
 *
 * All values use the same units as the sensor conversion routines:
 *   - temp_min_x100 / temp_max_x100      : 0.01 °C
 *   - rh_min_x100 / rh_max_x100          : 0.01 %RH
 *   - moisture_min_mv / moisture_max_mv  : millivolts (from SEN0114)
 *   - lux_min / lux_max                  : estimated lux (int32_t)
 *
 * This structure is stored as-is in NVS via the plant_profile_* API.
 */
struct __attribute__((__packed__)) plant_profile {
    int32_t temp_min_x100;
    int32_t temp_max_x100;

    int32_t rh_min_x100;
    int32_t rh_max_x100;

    int32_t moisture_min_mv;
    int32_t moisture_max_mv;

    int32_t lux_min;
    int32_t lux_max;
};

/* ------------------------------------------------------------------------- */
/* Log records                                                               */
/* ------------------------------------------------------------------------- */

/**
 * @brief Compact on-flash record for periodic sensor logging.
 *
 * This structure is written directly to NVS. It contains:
 *   - Timestamp in seconds since boot or epoch
 *   - Scaled temperature and humidity values
 *   - Raw 32-bit lux reading
 *   - Moisture reading in millivolts
 *
 * Conversion to human-friendly units (°C, %RH, etc.) is typically done
 * at print / decode time.
 */
struct __attribute__((__packed__)) log_record {
    uint32_t ts_s;        /**< Timestamp in seconds */
    int32_t  t_raw;       /**< Temperature scaled (e.g. x0.01 °C) */
    int32_t  rh_raw;      /**< Humidity scaled (e.g. x0.01 %RH) */
    uint32_t lux_raw;     /**< Raw 32-bit lux value */
    int32_t  moisture_mv; /**< Moisture reading in millivolts */
};

/**
 * @brief Iterator for walking log records from oldest to newest.
 *
 * Used with log_iter_begin() and log_iter_next() to traverse the
 * ring-buffer style sequence of log records stored in NVS.
 */
struct log_iter {
    uint16_t cur;      /**< Current key */
    uint16_t end;      /**< Last (most recent) key */
    uint8_t  wrapped;  /**< Internal flag: ring has wrapped at least once */
    uint8_t  started;  /**< Internal flag: iteration has started */
};

/* ------------------------------------------------------------------------- */
/* Log storage API                                                           */
/* ------------------------------------------------------------------------- */

/**
 * @brief Initialize NVS-backed log storage.
 *
 * Locates the "storage" partition via Partition Manager and mounts NVS
 * on top of it. This must be called before any other log_store_* or
 * plant_profile_* API is used.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int log_store_init(void);

/**
 * @brief Compute approximate capacity in number of records.
 *
 * Returns an estimate of how many records of the given size can be stored
 * in the configured NVS partition used for log records.
 *
 * @param record_size Size of each record in bytes
 *                    (typically sizeof(struct log_record)).
 *
 * @return Approximate number of records that can be stored, or -1 on error.
 */
size_t log_store_capacity_records(size_t record_size);

/**
 * @brief Append one log record to flash.
 *
 * Writes a single log_record into NVS, advancing the internal ring-buffer
 * sequence index. If @p out_key is non-NULL, the NVS key used is returned
 * to the caller.
 *
 * @param rec      Pointer to the record to append.
 * @param out_key  Optional pointer to receive the NVS key used.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int log_store_append(const struct log_record *rec, uint16_t *out_key);

/**
 * @brief Read the most recent log record.
 *
 * Reads the newest available record from NVS into @p out. If @p out_key
 * is non-NULL, the corresponding NVS key is also returned.
 *
 * @param out      Pointer to buffer to receive the latest record.
 * @param out_key  Optional pointer to receive the NVS key of the record.
 *
 * @return 0 on success, -ENOENT if no records exist, or another
 *         negative errno-style code on error.
 */
int log_store_read_latest(struct log_record *out, uint16_t *out_key);

/**
 * @brief Read a log record by NVS key.
 *
 * Attempts to read a record stored at the provided NVS key.
 *
 * @param key  NVS key of the record to read.
 * @param out  Pointer to buffer to receive the record.
 *
 * @return 0 on success, -ENOENT if the key does not contain a record,
 *         or another negative errno-style code on error.
 */
int log_store_read_key(uint16_t key, struct log_record *out);

/**
 * @brief Initialize an iterator to walk all records.
 *
 * Prepares @p it to iterate from the oldest valid record to the newest
 * using log_iter_next().
 *
 * @param it Pointer to iterator state to initialize.
 *
 * @return 0 on success, -ENOENT if the log is empty, or negative
 *         errno-style code on error.
 */
int log_iter_begin(struct log_iter *it);

/**
 * @brief Fetch the next record using an iterator.
 *
 * Reads the next available record indicated by @p it. If @p out_key
 * is non-NULL, the NVS key for the returned record is also provided.
 * The iterator automatically skips empty / invalid slots.
 *
 * @param it      Pointer to iterator state.
 * @param out     Pointer to buffer to receive the next record.
 * @param out_key Optional pointer to receive the record's NVS key.
 *
 * @return 0 on success, -ENOENT when no more records are available,
 *         or negative errno-style code on error.
 */
int log_iter_next(struct log_iter *it,
                  struct log_record *out,
                  uint16_t *out_key);

/**
 * @brief Erase and reformat the log storage partition.
 *
 * Deletes all stored log records by erasing the underlying NVS area and
 * resetting internal state.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int log_store_format(void);

/**
 * @brief Format a single log_record into a printf-style line.
 *
 * Converts @p rec and its @p key into a human-readable textual
 * representation suitable for logging or printing.
 *
 * The actual output is implementation-defined but generally includes:
 *   - key
 *   - timestamp
 *   - temperature / humidity
 *   - lux and moisture values
 *
 * @param rec Pointer to the record to format.
 * @param key NVS key corresponding to the record.
 */
void log_store_format_record(const struct log_record *rec, uint16_t key);

/**
 * @brief Dump all log records to stdout using printf.
 *
 * Iterates over the entire log (from oldest to newest) and prints each
 * record via log_store_format_record().
 *
 * @return 0 on success, -ENOENT if no records exist, or another
 *         negative errno-style code on error.
 */
int log_store_dump_to_printf(void);

/* ------------------------------------------------------------------------- */
/* Plant profile API                                                         */
/* ------------------------------------------------------------------------- */

/**
 * @brief Save the plant profile to NVS.
 *
 * Overwrites any previously stored profile with the contents of @p p.
 *
 * @param p Pointer to the plant profile to persist.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int plant_profile_save(const struct plant_profile *p);

/**
 * @brief Load plant profile from NVS.
 *
 * Attempts to read a previously-saved plant profile into @p out.
 *
 * @param out Pointer to buffer to receive the loaded profile.
 *
 * @return 0 on success (profile loaded into @p out);
 *         -ENOENT if no profile has been stored yet;
 *         other negative errno-style code on error.
 */
int plant_profile_load(struct plant_profile *out);

/**
 * @brief Load plant profile, or create and save a default.
 *
 * Tries to load an existing profile via plant_profile_load(). If none
 * exists (-ENOENT), a default profile is created, saved to NVS, and
 * returned via @p out.
 *
 * On successful return, @p out always contains a valid profile.
 *
 * @param out Pointer to buffer to receive the loaded or default profile.
 *
 * @return 0 on success (profile loaded or default created and saved);
 *         negative errno-style code on error (e.g., NVS/flash issues).
 */
int plant_profile_load_or_init(struct plant_profile *out);

#ifdef __cplusplus
}
#endif

#endif /* LOG_STORE_H */
