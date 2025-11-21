#ifndef SPMS_SENSORS_H
#define SPMS_SENSORS_H

/**
 * @file
 * @brief Unified sensor subsystem for the SPMS project.
 *
 * Provides initialization and update routines for all sensors in the
 * Smart Plant Monitoring System, including:
 *   - SEN0390 ambient light sensor
 *   - SEN0546 (CHT832X) temperature and humidity sensor
 *   - SEN0114 moisture sensor (ADC-based)
 *
 * The subsystem maintains a filtered light value, performs unit
 * conversions, and optionally evaluates readings against a stored
 * plant profile.
 */

#include <stdint.h>
#include "log_store.h"   /* For struct plant_profile */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Snapshot of the latest processed sensor readings.
 *
 * All values are expressed in engineering units suitable for logging,
 * BLE transmission, and plant-profile comparisons.
 */
struct spms_sensor_snapshot {
    int16_t  temp_c_x100;     /**< Temperature in 0.01 °C units */
    uint16_t rh_x100;         /**< Relative humidity in 0.01 %RH units */
    uint32_t lux_raw;         /**< Raw lux code from the SEN0390 sensor */
    uint32_t lux_filtered;    /**< Low-pass filtered lux value */
    int32_t  lux_est;         /**< Estimated illuminance in lux */
    int32_t  moisture_mv;     /**< Moisture reading in millivolts */
};

/**
 * @brief Initialize the sensor subsystem.
 *
 * This routine:
 *   - Binds all sensors from devicetree
 *   - Verifies that the I2C buses and peripheral devices are ready
 *   - Initializes the SEN0114 ADC channel
 *   - Resets internal filtering state
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int spms_sensors_init(void);

/**
 * @brief Read all sensors, update internal filtering, and generate a snapshot.
 *
 * This routine:
 *   - Reads the SEN0390 ambient light sensor
 *   - Updates the low-pass filtered lux value
 *   - Reads temperature & humidity from the SEN0546 sensor
 *   - Converts sensor data into engineering units
 *   - Estimates lux using sen0390_raw_to_lux()
 *   - Reads soil moisture from the SEN0114 ADC sensor
 *   - Populates the provided snapshot struct
 *   - Compares readings against the given plant profile, if provided
 *
 * @param profile Pointer to plant profile thresholds. May be NULL to skip checks.
 * @param out     Pointer to the snapshot struct to populate.
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int spms_sensors_update(const struct plant_profile *profile,
                        struct spms_sensor_snapshot *out);

#ifdef __cplusplus
}
#endif

#endif /* SPMS_SENSORS_H */
