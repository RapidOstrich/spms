#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#define CHT_NODE DT_NODELABEL(cht832x)
static const struct i2c_dt_spec cht_i2c = I2C_DT_SPEC_GET(CHT_NODE);

int main(void) {
    if (!device_is_ready(cht_i2c.bus)) {
        printk("I2C bus %s is not ready!\n\r", cht_i2c.bus->name);
        return;
    }

    while (1) {
        /*
        CHT832X one-shot (clock-stretch enabled):
        Returns 6 bytes in this order:
        Temp MSB, Temp LSB, Temp CRC, RH MSB, RH LSB, RH CRC
        */
        uint8_t oneshot_opcode[2] = {0x2C, 0x06};
        uint8_t read_buf[6];

        int ret = i2c_write_read_dt(
                &cht_i2c,
                oneshot_opcode,
                sizeof(oneshot_opcode),
                read_buf,
                sizeof(read_buf)
        );
        if (ret) {
            printk("I2C transfer failed: addr=%#04x cmd=%#04x%02x err=%d\n",
                   cht_i2c.addr, oneshot_opcode[0], oneshot_opcode[1], ret);
            k_sleep(K_MSEC(500));
            continue;
        }

        // Combine 2-byte value pairs
        uint16_t raw_temp  = ((uint16_t)read_buf[0] << 8) | read_buf[1];
        uint16_t raw_rh = ((uint16_t)read_buf[3] << 8) | read_buf[4];
        // Calculations
        float temp_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
        float rh_percent = 100.0f * ((float)raw_rh / 65535.0f);
        // Convert to fixed point (alternatively, enable FPU)
        int final_temp = (int)(temp_c * 100);
        int final_rh = (int)(rh_percent * 100);

        printk("T=%d.%02d C, RH=%d.%02d %%\n",
                final_temp / 100, final_temp % 100,
                final_rh / 100,   final_rh % 100
        );

        k_sleep(K_SECONDS(1));
    }
    return 0;
}
