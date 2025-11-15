#ifndef SEN0390_H
#define SEN0390_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read 32-bit light value from SEN0390 ambient light sensor.
 *
 * This performs a single I2C read of 4 bytes from the sensor and
 * combines them into a 32-bit raw value. The exact scaling from
 * raw counts to lux depends on the sensor's internal configuration,
 * but you can typically treat this as a lux-like value.
 *
 * @param i2c      Pointer to I2C device specification (from devicetree).
 * @param lux_raw  Output pointer for the 32-bit raw light value.
 *
 * @return 0 on success, negative errno code on failure.
 */
int query_sen0390(const struct i2c_dt_spec *i2c, uint32_t *lux_raw);

#ifdef __cplusplus
}
#endif

#endif /* SEN0390_H */
