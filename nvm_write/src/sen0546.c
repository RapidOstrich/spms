#include "sen0546.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int query_sen0546(
    const struct i2c_dt_spec *i2c,
    uint16_t *temp_raw,
    uint16_t *rh_raw
) {
    uint8_t oneshot_opcode[2] = {0x2C, 0x06};
    uint8_t read_buf[6];
    int ret;

    if (!device_is_ready(i2c->bus)) {
        printk("I2C bus %s not ready!\n", i2c->bus->name);
        return -ENODEV;
    }

    ret = i2c_write_read_dt(
        i2c,
        oneshot_opcode,
        sizeof(oneshot_opcode),
        read_buf,
        sizeof(read_buf)
    );

    if (ret < 0) {
        printk("I2C transfer failed: addr=%#04x err=%d\n", i2c->addr, ret);
        return ret;
    }

    *temp_raw = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    *rh_raw   = ((uint16_t)read_buf[3] << 8) | read_buf[4];

    return 0;
}
