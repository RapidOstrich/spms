#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include "button_dump.h"
#include "log_store.h"

#if !DT_NODE_EXISTS(DT_ALIAS(sw0))
#error "Board/overlay must define alias 'sw0' for the dump button"
#endif

static const struct gpio_dt_spec dump_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback dump_btn_cb;
static struct k_work dump_work;

static void dump_work_handler(struct k_work *work) {
    static uint32_t last_ms;
    uint32_t now = k_uptime_get_32();
    if ((now - last_ms) < 150) {
        return;
    }
    last_ms = now;

    log_store_dump_to_printf();
}

static void dump_btn_isr(
    const struct device *port,
    struct gpio_callback *cb,
    uint32_t pins
) {
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_work_submit(&dump_work);
}

int dump_button_init(void) {
    int rc;

    if (!device_is_ready(dump_btn.port)) {
        return -ENODEV;
    }

    rc = gpio_pin_configure_dt(&dump_btn, GPIO_INPUT);
    if (rc) {
        return rc;
    }

    rc = gpio_pin_interrupt_configure_dt(&dump_btn, GPIO_INT_EDGE_TO_ACTIVE);
    if (rc) {
        return rc;
    }
    
    gpio_init_callback(&dump_btn_cb, dump_btn_isr, BIT(dump_btn.pin));
    gpio_add_callback(dump_btn.port, &dump_btn_cb);

    k_work_init(&dump_work, dump_work_handler);
    return 0;
}
