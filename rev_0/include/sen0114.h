#ifndef SEN0114_H
#define SEN0114_H

/**
 * @file
 * @brief SEN0114 moisture sensor (ADC-based) interface.
 *
 * Provides initialization and one-shot sampling routines for the
 * SEN0114 soil moisture sensor using Zephyr's ADC API.
 */

#include <stdint.h>
#include <zephyr/drivers/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ADC channel used by the SEN0114 moisture sensor.
 *
 * This must be called once at startup before any sample is taken.
 * The function configures the ADC channel using `adc_channel_setup_dt()`
 * and performs a sanity-check read to verify communication.
 *
 * @param adc_dt Pointer to the adc_dt_spec for the configured ADC channel.
 * @return 0 on success, negative errno on failure.
 */
int sen0114_init(const struct adc_dt_spec *adc_dt);

/**
 * @brief Read a raw ADC sample from the SEN0114 moisture sensor.
 *
 * Reads a single signed 16-bit ADC conversion result from the sensor's ADC
 * channel using `adc_read_dt()`.
 *
 * @param adc_dt Pointer to the adc_dt_spec for this ADC channel.
 * @param raw    Pointer to store the raw ADC value.
 * @return 0 on success, negative errno on failure.
 */
int query_sen0114_raw(const struct adc_dt_spec *adc_dt, int16_t *raw);

/**
 * @brief Read the current moisture value in millivolts.
 *
 * Reads a raw ADC value and converts it to millivolts using
 * `adc_raw_to_millivolts_dt()`, based on the devicetree configuration
 * (reference voltage, gain, acquisition time, etc.).
 *
 * @param adc_dt Pointer to the adc_dt_spec for this ADC channel.
 * @param mv     Pointer to store the computed moisture level in millivolts.
 * @return 0 on success, negative errno on failure.
 */
int query_sen0114_mv(const struct adc_dt_spec *adc_dt, int *mv);

#ifdef __cplusplus
}
#endif

#endif /* SEN0114_H */
