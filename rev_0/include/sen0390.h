#ifndef SEN0390_H
#define SEN0390_H

/**
 * @file
 * @brief SEN0390 ambient light sensor interface.
 *
 * Declares a minimal API for reading the raw 32-bit light register from
 * the SEN0390 ambient light sensor over I2C, and converting that raw
 * value to an approximate illuminance in lux.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read a 32-bit raw light value from the SEN0390 sensor.
 *
 * Performs a single I2C transaction to read 4 bytes from the sensor's
 * light value register block (0x00–0x03) and combines them into a
 * 32-bit value.
 *
 * The raw output is sensor-specific and not yet in lux; use
 * sen0390_raw_to_lux() to convert to an approximate lux value.
 *
 * @param i2c     Pointer to I2C device specification (from devicetree).
 * @param lux_raw Output pointer for the 32-bit raw light value.
 *
 * @return 0 on success, negative errno code on failure.
 */
int query_sen0390(const struct i2c_dt_spec *i2c, uint32_t *lux_raw);

/**
 * @brief Convert a raw SEN0390 reading into an estimated lux value.
 *
 * Applies a simple linear calibration between two reference points
 * (R_A → L_A, R_B → L_B) to translate the raw reading from query_sen0390()
 * into an approximate illuminance in lux.
 *
 * The calibration constants are defined in @c sen0390.c and should be
 * tuned using real-world measurements (e.g. a handheld lux meter) for
 * accurate results.
 *
 * @param lux_raw Raw reading returned by query_sen0390().
 * @return Estimated illuminance in lux.
 */
int32_t sen0390_raw_to_lux(uint32_t lux_raw);

#ifdef __cplusplus
}
#endif

#endif /* SEN0390_H */
