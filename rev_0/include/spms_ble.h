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
 * @brief 1 Hz periodic Bluetooth maintenance hook.
 *
 * Call this function once per second. It:
 *   - Increments a debug counter and sends notification if enabled
 *   - Pushes the latest temperature and humidity values to the BLE client
 *     if the client has subscribed to those characteristics
 *
 * @param temp_c_x100 Latest temperature in 0.01 °C units.
 * @param rh_x100     Latest relative humidity in 0.01 %RH units.
 */
void spms_ble_tick_1s(int16_t temp_c_x100, uint16_t rh_x100);

#ifdef __cplusplus
}
#endif

#endif /* SPMS_BLE_H */
