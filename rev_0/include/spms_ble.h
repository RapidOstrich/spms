#ifndef SPMS_BLE_H
#define SPMS_BLE_H

/**
 * @file
 * @brief Bluetooth interface for the Smart Plant Monitoring System (SPMS).
 *
 * This module initializes the Bluetooth stack, registers all SPMS GATT
 * services, and provides periodic maintenance for notifications.
 *
 * It exposes:
 *   - A debug characteristic (optional notifications)
 *   - Temperature and humidity characteristics (notify when subscribed)
 *   - Plant profile update characteristic (write from phone app)
 *
 * The module also supports an optional connection-indicator LED.
 */

#include <stdint.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Bluetooth and all SPMS GATT services.
 *
 * This routine:
 *   - Enables the Zephyr Bluetooth stack
 *   - Registers the main SPMS service (debug, temperature, humidity)
 *   - Registers the plant profile GATT service
 *   - Starts advertising with the configured device name
 *   - Optionally manages a connection-status LED
 *
 * @param conn_led Pointer to a GPIO DT descriptor for a connection LED.
 *                 May be NULL if no connection LED is desired.
 *
 * @return 0 on success, negative errno-style code on failure.
 */
int spms_ble_init(const struct gpio_dt_spec *conn_led);

/**
 * @brief Periodic 1 Hz BLE maintenance.
 *
 * Call this once per second from the main loop. It:
 *   - Updates cached sensor values used by GATT reads
 *   - Increments a debug counter and sends notifications if enabled
 *   - Notifies the latest sensor values if the client has
 *     subscribed to those characteristics.
 *
 * @param temp_c_x100 Latest temperature in 0.01 °C units.
 * @param rh_x100     Latest relative humidity in 0.01 %RH units.
 * @param lux_est     Latest estimated illuminance in lux.
 * @param moisture_mv Latest soil moisture in millivolts.
 */
void spms_ble_tick_1s(int16_t  temp_c_x100,
                      uint16_t rh_x100,
                      int32_t  lux_est,
                      int32_t  moisture_mv);

#ifdef __cplusplus
}
#endif

#endif /* SPMS_BLE_H */
