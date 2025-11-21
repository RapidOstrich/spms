#ifndef SEN0546_H
#define SEN0546_H

/**
 * @file
 * @brief SEN0546 (CHT832X) temperature and humidity sensor interface.
 *
 * This header declares a minimal one-shot reading API for the SEN0546
 * / CHT832X digital temperature and humidity sensor over I2C.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perform a one-shot temperature and humidity measurement.
 *
 * Issues a single-measurement command to the SEN0546 / CHT832X sensor and
 * reads back the raw 16-bit temperature and humidity values.
 *
 * The raw values can be converted to engineering units with the formulas
 * from the CHT832X datasheet:
 *
 * - Temperature (°C):
 *     T = -45 + 175 * (temp_raw / 65535.0)
 * - Relative humidity (%RH):
 *     RH = 100 * (rh_raw / 65535.0)
 *
 * @param i2c      Pointer to I2C device specification (from devicetree).
 * @param temp_raw Pointer to store the 16-bit raw temperature value.
 * @param rh_raw   Pointer to store the 16-bit raw humidity value.
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int query_sen0546(const struct i2c_dt_spec *i2c,
                  uint16_t *temp_raw,
                  uint16_t *rh_raw);

#ifdef __cplusplus
}
#endif

#endif /* SEN0546_H */
