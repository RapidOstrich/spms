#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

#include "sen0114.h"

LOG_MODULE_REGISTER(SEN0114, LOG_LEVEL_INF);

int sen0114_init(const struct adc_dt_spec *adc_dt)
{
    int err;
    struct adc_sequence sequence = { 0 };
    int16_t dummy_buf;  /* just to satisfy adc_sequence_init_dt */

    if (adc_dt == NULL) {
        return -EINVAL;
    }

    if (!device_is_ready(adc_dt->dev)) {
        LOG_ERR("Moisture: ADC device not ready");
        return -ENODEV;
    }

    err = adc_channel_setup_dt(adc_dt);
    if (err < 0) {
        LOG_ERR("Moisture: adc_channel_setup_dt failed (err %d)", err);
        return err;
    }

    err = adc_sequence_init_dt(adc_dt, &sequence);
    if (err < 0) {
        LOG_ERR("Moisture: adc_sequence_init_dt failed (err %d)", err);
        return err;
    }

    sequence.buffer = &dummy_buf;
    sequence.buffer_size = sizeof(dummy_buf);

    /* Optional sanity check read */
    err = adc_read_dt(adc_dt, &sequence);
    if (err < 0) {
        LOG_ERR("Moisture: initial adc_read_dt failed (err %d)", err);
        return err;
    }

    LOG_INF("Moisture: ADC channel initialized");
    return 0;
}

int query_sen0114_raw(const struct adc_dt_spec *adc_dt, int16_t *raw)
{
    int err;
    struct adc_sequence sequence = { 0 };

    if ((adc_dt == NULL) || (raw == NULL)) {
        return -EINVAL;
    }

    err = adc_sequence_init_dt(adc_dt, &sequence);
    if (err < 0) {
        LOG_ERR("Moisture: adc_sequence_init_dt failed (err %d)", err);
        return err;
    }

    sequence.buffer = raw;
    sequence.buffer_size = sizeof(*raw);

    err = adc_read_dt(adc_dt, &sequence);
    if (err < 0) {
        LOG_ERR("Moisture: adc_read_dt failed (err %d)", err);
        return err;
    }

    return 0;
}

int query_sen0114_mv(const struct adc_dt_spec *adc_dt, int *mv)
{
    int err;
    int16_t raw;
    int val_mv;

    if ((adc_dt == NULL) || (mv == NULL)) {
        return -EINVAL;
    }

    err = query_sen0114_raw(adc_dt, &raw);
    if (err < 0) {
        LOG_ERR("Moisture: query_sen0114_raw failed (err %d)", err);
        return err;
    }

    val_mv = (int)raw;

    err = adc_raw_to_millivolts_dt(adc_dt, &val_mv);
    if (err < 0) {
        LOG_ERR("Moisture: adc_raw_to_millivolts_dt failed (err %d)", err);
        return err;
    }

    *mv = val_mv;
    return 0;
}
