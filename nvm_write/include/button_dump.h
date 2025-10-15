#ifndef BUTTON_DUMP_H
#define BUTTON_DUMP_H

/**
 * @brief Initialize the button interrupt that triggers a log dump.
 *
 * Configures the board's SW0 button to trigger a background job
 * that prints all stored sensor data through printf.
 *
 * @return 0 on success, negative errno on failure.
 */
int dump_button_init(void);

#endif // BUTTON_DUMP_H
