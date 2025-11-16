#ifndef SPMS_SENSORS_H
#define SPMS_SENSORS_H

#include <stdint.h>
#include "log_store.h"   /* for struct plant_profile */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Snapshot of the latest sensor readings.
 *
 * All values are already converted into engineering units suitable
 * for logging, BLE, and plant-profile comparisons.
 */
struct spms_sensor_snapshot {
    int16_t  temp_c_x100;     /**< Temperature in 0.01 °C units */
    uint16_t rh_x100;         /**< Relative humidity in 0.01 %RH units */
    uint32_t lux_raw;         /**< Latest raw lux code from SEN0390 */
    uint32_t lux_filtered;    /**< Filtered lux code (for decisions) */
    int32_t  lux_est;         /**< Estimated illuminance in lux */
};

/**
 * @brief Initialize sensor subsystem (I2C bindings, basic checks).
 *
 * This function:
 *   - Verifies that the I2C bus and sensor devices are ready.
 *   - Logs basic information such as I2C addresses.
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int spms_sensors_init(void);

/**
 * @brief Read sensors, update internal filters, and compare against profile.
 *
 * This function:
 *   - Reads the SEN0390 ambient light sensor
 *   - Updates the internal filtered lux value
 *   - Reads the SEN0546 temperature/humidity sensor
 *   - Converts readings to engineering units
 *   - Estimates lux using sen0390_raw_to_lux()
 *   - Logs a summary line with the current readings
 *   - Compares against the provided plant profile thresholds and logs
 *     warnings when values are out of range.
 *
 * @param profile Plant profile thresholds (may be NULL to skip comparisons).
 * @param out     Pointer to snapshot struct to receive the latest readings.
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int spms_sensors_update(const struct plant_profile *profile,
                        struct spms_sensor_snapshot *out);

#ifdef __cplusplus
}
#endif

#endif /* SPMS_SENSORS_H */
