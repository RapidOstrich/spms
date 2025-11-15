#ifndef SEN0114_H
#define SEN0114_H

#include <stdint.h>
#include <zephyr/drivers/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SEN0114 moisture sensor ADC channel.
 *
 * Must be called once at startup before any read.
 *
 * @param adc_dt Pointer to adc_dt_spec from devicetree (e.g. ADC_DT_SPEC_GET(...)).
 * @return 0 on success, negative errno on failure.
 */
int sen0114_init(const struct adc_dt_spec *adc_dt);

/**
 * @brief Read the current raw ADC value from the moisture sensor.
 *
 * @param adc_dt Pointer to adc_dt_spec from devicetree.
 * @param raw    Pointer to store raw ADC sample (signed 16-bit).
 * @return 0 on success, negative errno on failure.
 */
int query_sen0114_raw(const struct adc_dt_spec *adc_dt, int16_t *raw);

/**
 * @brief Read the current moisture as millivolts.
 *
 * Uses adc_raw_to_millivolts_dt() with your devicetree config.
 *
 * @param adc_dt Pointer to adc_dt_spec for this channel.
 * @param mv     Pointer to store value in millivolts.
 * @return 0 on success, negative errno on failure.
 */
int query_sen0114_mv(const struct adc_dt_spec *adc_dt, int *mv);

#ifdef __cplusplus
}
#endif

#endif /* SEN0114_H */
