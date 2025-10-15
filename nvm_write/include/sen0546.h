#ifndef SEN0546_H
#define SEN0546_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perform one-shot read from SEN0546 sensor.
 *
 * @param i2c Pointer to I2C device specification (from devicetree).
 * @param temp_raw Pointer to store 16-bit raw temperature value.
 * @param rh_raw Pointer to store 16-bit raw humidity value.
 * @return 0 on success, negative errno code on failure.
 */
int query_sen0546(const struct i2c_dt_spec *i2c, uint16_t *temp_raw, uint16_t *rh_raw);

#ifdef __cplusplus
}
#endif

#endif /* SEN0546_H */
