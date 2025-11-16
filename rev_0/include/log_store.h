#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Plant profile thresholds stored in NVS.
 *
 * All values are in the same units your sensors use:
 *   - temp_min_x100 / temp_max_x100      : 0.01 °C
 *   - rh_min_x100 / rh_max_x100          : 0.01 %RH
 *   - moisture_min_mv / moisture_max_mv  : millivolts (from SEN0114)
 *   - lux_min / lux_max                  : estimated lux (int32)
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

/**
 * @brief Compact on-flash record for periodic sensor logging.
 *
 * This structure is written directly to NVS and contains:
 *   - Timestamp in seconds
 *   - Scaled temperature and humidity values
 *   - Raw 32-bit lux reading
 *   - Moisture reading in millivolts
 *
 * Conversion to human-friendly units should be done when printing/decoding.
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
 * in the configured NVS partition.
 *
 * @param record_size Size of each record in bytes (typically sizeof(struct log_record)).
 *
 * @return Approximate number of records that can be stored.
 */
size_t log_store_capacity_records(size_t record_size);

/**
 * @brief Append one log record to flash.
 *
 * Writes a single log_record into NVS, advancing the internal sequence
 * (ring buffer) index. If @p out_key is non-NULL, the NVS key used is
 * returned to the caller.
 *
 * @param rec      Pointer to the record to write.
 * @param out_key  Optional; if non-NULL, receives the NVS key used.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int log_store_append(const struct log_record *rec, uint16_t *out_key);

/**
 * @brief Read the most recent log record.
 *
 * Retrieves the latest (most recently written) record from NVS. If
 * @p out_key is non-NULL, the key it was read from is returned.
 *
 * @param out      Pointer to buffer to receive the record.
 * @param out_key  Optional; if non-NULL, receives the record key.
 *
 * @return 0 on success;
 *         -ENOENT if there are no records stored;
 *         other negative errno-style code on error.
 */
int log_store_read_latest(struct log_record *out, uint16_t *out_key);

/**
 * @brief Read a specific record by key.
 *
 * Attempts to read the record associated with @p key from NVS into @p out.
 *
 * @param key  NVS key to read from.
 * @param out  Pointer to buffer to receive the record data.
 *
 * @return 0 on success;
 *         -ENOENT if no record exists at the given key;
 *         other negative errno-style code on error.
 */
int log_store_read_key(uint16_t key, struct log_record *out);

/**
 * @brief Begin iteration over all stored records from oldest to newest.
 *
 * Initializes @p it to iterate over the current set of log records in
 * chronological order. Use log_iter_next() to advance the iterator.
 *
 * @param it  Pointer to iterator structure to initialize.
 *
 * @return 0 on success;
 *         -ENOENT if there are no records to iterate over;
 *         other negative errno-style code on error.
 */
int log_iter_begin(struct log_iter *it);

/**
 * @brief Get the next record during iteration.
 *
 * Continues an iteration started by log_iter_begin(), returning the next
 * record and its key (if @p out_key is non-NULL).
 *
 * @param it       Pointer to iterator previously initialized by log_iter_begin().
 * @param out      Pointer to buffer to receive the next record.
 * @param out_key  Optional; if non-NULL, receives the record key.
 *
 * @return 0 on success (one record returned in @p out);
 *         -ENOENT when no more records are available;
 *         other negative errno-style code on error.
 */
int log_iter_next(struct log_iter *it, struct log_record *out, uint16_t *out_key);

/**
 * @brief Erase all stored records and reset sequence state.
 *
 * Formats the underlying NVS area used for logging and resets any
 * internal state so that subsequent appends start from a clean slate.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int log_store_format(void);

/**
 * @brief Format a log record for printing.
 *
 * Weak hook that can be overridden by the application to customize how
 * a log_record is printed when log_store_dump_to_printf() is called.
 *
 * @param rec  Pointer to the record being printed.
 * @param key  NVS key associated with this record.
 */
void log_store_format_record(const struct log_record *rec, uint16_t key);

/**
 * @brief Dump all stored records using printf/logging.
 *
 * Iterates over all records in chronological order and prints them using
 * log_store_format_record().
 *
 * @return Number of records printed on success;
 *         negative errno-style code on error (e.g., -ENOENT if empty).
 */
int log_store_dump_to_printf(void);

/**
 * @brief Save the current plant profile to NVS.
 *
 * Writes the provided plant profile structure to NVS so that it can be
 * restored on subsequent boots.
 *
 * @param p  Pointer to plant_profile data to save.
 *
 * @return 0 on success, negative errno-style code on error.
 */
int plant_profile_save(const struct plant_profile *p);

/**
 * @brief Load plant profile from NVS.
 *
 * Attempts to read a previously-saved plant profile from NVS into @p out.
 *
 * @param out  Pointer to buffer to receive the loaded profile.
 *
 * @return 0 on success (profile loaded into @p out);
 *         -ENOENT if no profile has been stored yet;
 *         other negative errno-style code on error.
 */
int plant_profile_load(struct plant_profile *out);

/**
 * @brief Load plant profile, or initialize defaults and save them.
 *
 * Attempts to load the plant profile from NVS. If none is found, a
 * default profile is initialized and saved back to NVS. On success,
 * @p out always contains a valid profile.
 *
 * @param out  Pointer to buffer to receive the loaded or default profile.
 *
 * @return 0 on success (profile loaded or default created and saved);
 *         negative errno-style code on error (e.g., NVS/flash issues).
 */
int plant_profile_load_or_init(struct plant_profile *out);

#ifdef __cplusplus
}
#endif

#endif /* LOG_STORE_H */
