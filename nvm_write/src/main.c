#include <zephyr/kernel.h>
#include <zephyr/posix/time.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include "log_store.h"
#include "sen0546.h"
#include "button_dump.h"

#define CHT_NODE DT_NODELABEL(cht832x)
static const struct i2c_dt_spec cht_i2c = I2C_DT_SPEC_GET(CHT_NODE);

extern const struct i2c_dt_spec cht_i2c;

static void sample_and_store(void) {
    uint16_t t_raw, rh_raw;
    if (query_sen0546(&cht_i2c, &t_raw, &rh_raw) == 0) {
        uint32_t ts = (uint32_t)time(NULL);

        if (ts == (uint32_t)-1) {
            ts = (uint32_t)(k_uptime_get() / 1000);
        }

        struct log_record rec = {
            .ts_s  = ts,
            .t_raw = t_raw,
            .rh_raw= rh_raw,
        };

        uint16_t key;
        int rc = log_store_append(&rec, &key);

        if (rc == 0) {
            printk("Log write OK at key 0x%04x\n", key);
        }
        else {
            printk("Log write failed: %d\n", rc);
        }
    }
}

int main(void) {
    
    printk("*** Build: nvm_write ***\n");

    // Setup flash storage access
    if (log_store_init() != 0) {
        printk("log_store init failed\n");
        return;
    }

    // Setup ISR for button 1
    int ret = dump_button_init();
    if (ret) {
        printk("dump_button_init failed: %d\n", ret);
    }

    // Capture loop
    while (1) {
        k_sleep(K_SECONDS(10));
        sample_and_store();
    }

    return 0;
}
