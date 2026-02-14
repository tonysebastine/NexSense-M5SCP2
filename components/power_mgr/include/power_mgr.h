/*
 * NexSense M5-DoorSensor — Power Manager
 *
 * Battery monitoring via AXP2101 PMIC and light sleep management.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Configuration ---------- */

#define BAT_LOW_THRESHOLD_MV 3300 /* Low battery warning threshold (mV) */
#define SLEEP_DURATION_MS 500     /* Default light sleep duration        */

/* ---------- API ---------- */

/**
 * @brief Initialize the power manager (Direct GPIO/ADC for M5StickC Plus 2).
 * @return ESP_OK on success.
 */
esp_err_t power_mgr_init(void);

/**
 * @brief Read battery voltage via ADC1_CH2.
 * @return Battery voltage in millivolts.
 */
uint16_t power_mgr_get_battery_mv(void);

/**
 * @brief Check if battery is below the low threshold.
 * @return true if battery voltage < BAT_LOW_THRESHOLD_MV.
 */
bool power_mgr_is_battery_low(void);

/**
 * @brief Enter light sleep until next sample period or button press.
 * @param[in] duration_ms  Duration to sleep in milliseconds.
 * @return ESP_OK after waking up.
 */
esp_err_t power_mgr_enter_sleep(uint32_t duration_ms);

/**
 * @brief Enable/disable the LCD backlight power via GPIO 27.
 * @param[in] enable  true to power on LCD.
 */
esp_err_t power_mgr_set_lcd_power(bool enable);

/**
 * @brief Control the Red LED (GPIO 19).
 * @param[in] on  true to turn LED ON (active low/high depending on wiring).
 */
void power_mgr_set_led(bool on);

/**
 * @brief Sound the passive buzzer (GPIO 2).
 * @param[in] duration_ms  Duration of the beep in milliseconds.
 */
void power_mgr_beep(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
