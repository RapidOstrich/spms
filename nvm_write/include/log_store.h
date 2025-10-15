#ifndef LOG_STORE_H
#define LOG_STORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Compact on-wire record.
 * Store raw sensor words + timestamp (seconds).
 * Convert to human units when reading/displaying.
 */
struct __attribute__((__packed__)) log_record {
    uint32_t ts_s;     /* timestamp in seconds (wall or monotonic) */
    uint16_t t_raw;    /* sensor raw temperature */
    uint16_t rh_raw;   /* sensor raw humidity */
};

/* Opaque handle (forward-declared). */
struct log_store;

/*
 * Create a singleton handle (no dynamic allocation).
 * Returns pointer to internal static instance after initialization.
 */
struct log_store *log_store_get(void);

/*
 * Initialize NVS-backed storage.
 * - Locates the 'storage' flash partition (Partition Manager or DTS).
 * - Mounts NVS.
 * Returns 0 on success, <0 on error.
 */
int log_store_init(void);

/*
 * Append one record.
 * - Automatically manages a monotonic sequence and key assignment.
 * - Persists the latest sequence in NVS so it survives resets.
 * If out_key != NULL, the written key is returned there.
 */
int log_store_append(const struct log_record *rec, uint16_t *out_key);

/*
 * Read the latest record that was written.
 * If out_key != NULL, returns the key it was stored under.
 * Returns 0 on success, -ENOENT if no records exist, or other <0 errors.
 */
int log_store_read_latest(struct log_record *out, uint16_t *out_key);

/*
 * Read a record by its key (returned by append or iterator).
 * Returns 0 on success, <0 on error (e.g., -ENOENT if not found).
 */
int log_store_read_key(uint16_t key, struct log_record *out);

/*
 * Iterate over records in write order (oldest → newest).
 * Usage:
 *   struct log_iter it; log_iter_begin(&it);
 *   while (log_iter_next(&it, &rec, &key) == 0) { ... }
 *
 * Iteration tolerates wrapped sequences.
 */
struct log_iter {
    uint16_t cur;          /* current key during iteration */
    uint16_t end;          /* last (latest) key */
    uint8_t  wrapped;      /* whether the sequence has wrapped at least once */
    uint8_t  started;      /* internal */
};

/* Begin an iteration. Returns 0 if there is at least one record, -ENOENT otherwise. */
int log_iter_begin(struct log_iter *it);

/*
 * Advance iterator and read the next record into *out.
 * If out_key != NULL, returns the record's key.
 * Returns 0 on success, -ENOENT when done, or <0 on read error.
 */
int log_iter_next(struct log_iter *it, struct log_record *out, uint16_t *out_key);

/*
 * Optional: erase/format the storage partition (dangerous).
 * Returns 0 on success.
 */
int log_store_format(void);

/*
 * Optional: estimate how many records fit (approximate, includes NVS overhead headroom).
 * Returns a lower-bound estimate; safe for sizing.
 */
size_t log_store_capacity_records(size_t record_size);

/**
 * Print all stored records to stdout (UART/RTT).
 * @return number of records printed, or <0 on error (e.g., -ENOENT if empty)
 */
int log_store_dump_to_printf(void);

/*
 * Print all stored records to stdout.
 * @return number of records printed or <0 on error.
*/
int log_store_dump_to_printf(void);


#ifdef __cplusplus
}
#endif

#endif /* LOG_STORE_H */
