#ifndef SPMS_BLE_H
#define SPMS_BLE_H

#include <zephyr/drivers/gpio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Bluetooth stack and SPMS GATT services.
 *
 * This function:
 *   - Enables the Bluetooth stack
 *   - Registers the SPMS data service (debug / temperature / humidity)
 *   - Registers the plant profile service
 *   - Starts connectable advertising with the configured device name
 *
 * @param conn_led Pointer to connection-indicator LED descriptor.
 *                 May be NULL if no LED indication is desired.
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int spms_ble_init(const struct gpio_dt_spec *conn_led);

/**
 * @brief Periodic 1 Hz BLE maintenance.
 *
 * Call this once per second from the main loop. It:
 *   - Increments a debug counter and sends notifications if enabled
 *   - Notifies the latest temperature and humidity if the client
 *     has subscribed to those characteristics.
 *
 * @param temp_c_x100 Latest temperature in 0.01 °C units.
 * @param rh_x100     Latest relative humidity in 0.01 %RH units.
 */
void spms_ble_tick_1s(int16_t temp_c_x100, uint16_t rh_x100);

#ifdef __cplusplus
}
#endif

#endif /* SPMS_BLE_H */
