/*
 * NexSense M5-DoorSensor — UI Display
 *
 * Minimal status display on the ST7789V2 LCD (135×240).
 */

#pragma once

#include "esp_err.h"
#include "state_engine.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Display Constants ---------- */

#define LCD_WIDTH 135
#define LCD_HEIGHT 240

/* ---------- API ---------- */

/**
 * @brief Initialize the ST7789V2 LCD display.
 * @return ESP_OK on success.
 */
esp_err_t ui_init(void);

/**
 * @brief Show the boot splash screen with device info.
 * @param[in] device_id   32-bit device ID.
 * @param[in] fw_version  Firmware version string.
 */
void ui_show_boot(uint32_t device_id, const char *fw_version);

/**
 * @brief Update the main status screen.
 * @param[in] state       Current door state.
 * @param[in] battery_mv  Battery voltage in mV.
 * @param[in] event_count Total events sent.
 * @param[in] calibrated  Whether device is calibrated.
 */
void ui_show_state(door_state_t state, uint16_t battery_mv,
                   uint32_t event_count, bool calibrated);

/**
 * @brief Show calibration mode screen.
 * @param[in] step  Description of current calibration step.
 */
void ui_show_calibration(const char *step);

/**
 * @brief Show a brief message and return.
 * @param[in] msg   Message text.
 * @param[in] color 16-bit RGB565 color.
 */
void ui_show_message(const char *msg, uint16_t color);

/**
 * @brief Turn display on/off (controls backlight only).
 */
void ui_set_display(bool on);

/**
 * @brief Check if display is currently active.
 */
bool ui_is_display_on(void);

/* ---------- Colors (RGB565) ---------- */

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0
#define COLOR_CYAN 0x07FF
#define COLOR_ORANGE 0xFD20
#define COLOR_DARK_BG 0x1082 /* Dark grey background */
#define COLOR_ACCENT 0x2D7F  /* Cyber-teal accent    */

#ifdef __cplusplus
}
#endif
