/*
 * NexSense M5-DoorSensor — UI Display
 *
 * ST7789V2 SPI LCD driver (135×240) for the M5StickC Plus 2.
 * Provides boot, status, calibration, and message screens.
 *
 * Hardware:
 *   - ST7789V2 on SPI2: CLK=GPIO13, MOSI=GPIO15, CS=GPIO5, DC=GPIO14,
 * RST=GPIO12
 *   - Backlight controlled via GPIO 27
 *   - Resolution: 135×240 (portrait orientation)
 */

#include "ui.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "power_mgr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* ---------- Constants ---------- */

static const char *TAG = "ui";

/* SPI pins */
#define LCD_SPI_HOST SPI2_HOST
#define LCD_PIN_CLK GPIO_NUM_13
#define LCD_PIN_MOSI GPIO_NUM_15
#define LCD_PIN_CS GPIO_NUM_5
#define LCD_PIN_DC GPIO_NUM_14
#define LCD_PIN_RST GPIO_NUM_12

/* ST7789V2 commands */
#define ST7789_SWRESET 0x01
#define ST7789_SLPOUT 0x11
#define ST7789_NORON 0x13
#define ST7789_INVON 0x21
#define ST7789_DISPON 0x29
#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_MADCTL 0x36
#define ST7789_COLMOD 0x3A

/* Display offset (ST7789V2 135×240 has column offset of 52, row offset of 40)
 */
#define COL_OFFSET 52
#define ROW_OFFSET 40

/* Display timeout */
#define DISPLAY_TIMEOUT_MS 10000

/* ---------- State ---------- */

static spi_device_handle_t s_spi = NULL;
static bool s_display_on = false;
static int64_t s_last_activity_us = 0;

/* Simple 5×7 font bitmap (ASCII 32-127, basic) */
/* Only supports uppercase + digits + common punctuation for minimal footprint
 */
#include "font5x7.h"

/* ---------- Low-Level SPI ---------- */

static void lcd_cmd(uint8_t cmd) {
  spi_transaction_t t = {
      .length = 8,
      .tx_buffer = &cmd,
  };
  gpio_set_level(LCD_PIN_DC, 0); /* Command mode */
  spi_device_transmit(s_spi, &t);
}

static void lcd_data(const uint8_t *data, size_t len) {
  if (len == 0)
    return;
  spi_transaction_t t = {
      .length = len * 8,
      .tx_buffer = data,
  };
  gpio_set_level(LCD_PIN_DC, 1); /* Data mode */
  spi_device_transmit(s_spi, &t);
}

static void lcd_data_byte(uint8_t val) { lcd_data(&val, 1); }

/* ---------- ST7789 Helpers ---------- */

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t data[4];

  x0 += COL_OFFSET;
  x1 += COL_OFFSET;
  y0 += ROW_OFFSET;
  y1 += ROW_OFFSET;

  lcd_cmd(ST7789_CASET);
  data[0] = x0 >> 8;
  data[1] = x0 & 0xFF;
  data[2] = x1 >> 8;
  data[3] = x1 & 0xFF;
  lcd_data(data, 4);

  lcd_cmd(ST7789_RASET);
  data[0] = y0 >> 8;
  data[1] = y0 & 0xFF;
  data[2] = y1 >> 8;
  data[3] = y1 & 0xFF;
  lcd_data(data, 4);

  lcd_cmd(ST7789_RAMWR);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                          uint16_t color) {
  if (w == 0 || h == 0)
    return;
  lcd_set_window(x, y, x + w - 1, y + h - 1);

  /* Use DMA-capable heap buffer for fast transfer */
  uint16_t *buf = heap_caps_malloc(w * h * 2, MALLOC_CAP_DMA);
  if (!buf) {
    /* Fallback: send line by line from stack */
    uint8_t line[LCD_WIDTH * 2];
    uint8_t hi = color >> 8, lo = color & 0xFF;
    for (int i = 0; i < w; i++) {
      line[i * 2] = hi;
      line[i * 2 + 1] = lo;
    }
    gpio_set_level(LCD_PIN_DC, 1);
    for (int row = 0; row < h; row++) {
      spi_transaction_t t = {.length = w * 16, .tx_buffer = line};
      spi_device_transmit(s_spi, &t);
    }
    return;
  }

  /* Byte-swap for SPI (big-endian) */
  uint16_t swapped = (color >> 8) | (color << 8);
  for (int i = 0; i < w * h; i++)
    buf[i] = swapped;

  gpio_set_level(LCD_PIN_DC, 1);
  spi_transaction_t t = {.length = w * h * 16, .tx_buffer = buf};
  spi_device_transmit(s_spi, &t);
  free(buf);
}

static void lcd_fill_screen(uint16_t color) {
  lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

/* ---------- Text Rendering ---------- */

static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg,
                          uint16_t bg, uint8_t scale) {
  if (c < 32 || c > 127)
    c = '?';
  int idx = c - 32;

  uint8_t char_w = 5 * scale;
  uint8_t char_h = 7 * scale;

  lcd_set_window(x, y, x + char_w - 1, y + char_h - 1);

  uint8_t buf[5 * 7 * 2 * 4]; /* Max scale 4 */
  gpio_set_level(LCD_PIN_DC, 1);

  for (int row = 0; row < 7; row++) {
    /* Build one scaled row */
    int pos = 0;
    for (int col = 0; col < 5; col++) {
      uint16_t color = (font5x7[idx][col] & (1 << row)) ? fg : bg;
      uint8_t hi = color >> 8;
      uint8_t lo = color & 0xFF;
      for (int s = 0; s < scale; s++) {
        buf[pos++] = hi;
        buf[pos++] = lo;
      }
    }
    /* Repeat row for vertical scaling */
    for (int s = 0; s < scale; s++) {
      spi_transaction_t t = {
          .length = pos * 8,
          .tx_buffer = buf,
      };
      spi_device_transmit(s_spi, &t);
    }
  }
}

static void lcd_draw_string(uint16_t x, uint16_t y, const char *str,
                            uint16_t fg, uint16_t bg, uint8_t scale) {
  while (*str) {
    lcd_draw_char(x, y, *str, fg, bg, scale);
    x += 6 * scale; /* 5px char + 1px spacing */
    str++;
  }
}

/* Center a string horizontally */
static uint16_t center_x(const char *str, uint8_t scale) {
  int len = strlen(str);
  int text_w = len * 6 * scale - scale; /* Remove last spacing */
  if (text_w >= LCD_WIDTH)
    return 0;
  return (LCD_WIDTH - text_w) / 2;
}

/* ---------- API: Initialization ---------- */

esp_err_t ui_init(void) {
  esp_err_t ret;

  /* Configure DC and RST pins */
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

  /* SPI bus configuration */
  spi_bus_config_t buscfg = {
      .mosi_io_num = LCD_PIN_MOSI,
      .miso_io_num = -1,
      .sclk_io_num = LCD_PIN_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2,
  };
  ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* SPI device */
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 20 * 1000 * 1000, /* 20 MHz */
      .mode = 0,
      .spics_io_num = LCD_PIN_CS,
      .queue_size = 7,
  };
  ret = spi_bus_add_device(LCD_SPI_HOST, &devcfg, &s_spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Hardware reset */
  gpio_set_level(LCD_PIN_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(LCD_PIN_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(120));

  /* ST7789V2 initialization sequence */
  lcd_cmd(ST7789_SWRESET);
  vTaskDelay(pdMS_TO_TICKS(150));

  lcd_cmd(ST7789_SLPOUT);
  vTaskDelay(pdMS_TO_TICKS(120));

  lcd_cmd(ST7789_COLMOD);
  lcd_data_byte(0x55); /* 16-bit color (RGB565) */

  lcd_cmd(ST7789_MADCTL);
  lcd_data_byte(0x00); /* Portrait, RGB order */

  lcd_cmd(ST7789_INVON); /* Display inversion on (required for ST7789) */

  lcd_cmd(ST7789_NORON);
  vTaskDelay(pdMS_TO_TICKS(10));

  lcd_cmd(ST7789_DISPON);
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Enable backlight */
  power_mgr_set_lcd_power(true);
  s_display_on = true;
  s_last_activity_us = esp_timer_get_time();

  /* Clear screen */
  lcd_fill_screen(COLOR_DARK_BG);

  ESP_LOGI(TAG, "LCD initialized (135x240 ST7789V2)");
  return ESP_OK;
}

/* ---------- API: Screens ---------- */

void ui_show_boot(uint32_t device_id, const char *fw_version) {
  s_last_activity_us = esp_timer_get_time();

  lcd_fill_screen(COLOR_DARK_BG);

  /* Logo / Title */
  const char *title = "NEXSENSE";
  lcd_draw_string(center_x(title, 2), 30, title, COLOR_ACCENT, COLOR_DARK_BG,
                  2);

  /* Subtitle */
  const char *sub = "DOOR SENSOR";
  lcd_draw_string(center_x(sub, 1), 60, sub, COLOR_WHITE, COLOR_DARK_BG, 1);

  /* Horizontal rule */
  lcd_fill_rect(10, 78, LCD_WIDTH - 20, 1, COLOR_ACCENT);

  /* Device ID */
  char id_str[20];
  snprintf(id_str, sizeof(id_str), "ID: %08lX", (unsigned long)device_id);
  lcd_draw_string(center_x(id_str, 1), 90, id_str, COLOR_CYAN, COLOR_DARK_BG,
                  1);

  /* Firmware version */
  char ver_str[24];
  snprintf(ver_str, sizeof(ver_str), "FW: %s", fw_version);
  lcd_draw_string(center_x(ver_str, 1), 105, ver_str, COLOR_CYAN, COLOR_DARK_BG,
                  1);

  /* Bottom accent */
  lcd_fill_rect(10, 220, LCD_WIDTH - 20, 2, COLOR_ACCENT);
}

void ui_show_state(door_state_t state, uint16_t battery_mv,
                   uint32_t event_count, bool calibrated) {
  s_last_activity_us = esp_timer_get_time();

  lcd_fill_screen(COLOR_DARK_BG);

  /* Header */
  lcd_fill_rect(0, 0, LCD_WIDTH, 20, COLOR_ACCENT);
  const char *hdr = "NEXSENSE";
  lcd_draw_string(center_x(hdr, 1), 6, hdr, COLOR_DARK_BG, COLOR_ACCENT, 1);

  if (!calibrated) {
    /* Not calibrated warning */
    const char *warn = "NOT";
    lcd_draw_string(center_x(warn, 3), 60, warn, COLOR_ORANGE, COLOR_DARK_BG,
                    3);
    const char *warn2 = "CALIBRATED";
    lcd_draw_string(center_x(warn2, 2), 95, warn2, COLOR_ORANGE, COLOR_DARK_BG,
                    2);

    const char *hint = "HOLD BTN A";
    lcd_draw_string(center_x(hint, 1), 140, hint, COLOR_WHITE, COLOR_DARK_BG,
                    1);
    return;
  }

  /* Main state display */
  const char *state_str = door_state_to_str(state);
  uint16_t state_color;
  switch (state) {
  case DOOR_OPEN:
    state_color = COLOR_RED;
    break;
  case DOOR_CLOSED:
    state_color = COLOR_GREEN;
    break;
  default:
    state_color = COLOR_YELLOW;
    break;
  }

  /* State indicator block */
  lcd_fill_rect(10, 35, LCD_WIDTH - 20, 60, state_color);
  lcd_draw_string(center_x(state_str, 3), 50, state_str, COLOR_BLACK,
                  state_color, 3);

  /* Separator */
  lcd_fill_rect(10, 105, LCD_WIDTH - 20, 1, COLOR_ACCENT);

  /* Battery */
  char bat_str[16];
  int bat_pct = 0;
  if (battery_mv >= 4200)
    bat_pct = 100;
  else if (battery_mv <= 3300)
    bat_pct = 0;
  else
    bat_pct = (battery_mv - 3300) * 100 / 900;

  snprintf(bat_str, sizeof(bat_str), "BAT: %d%%", bat_pct);
  uint16_t bat_color = (bat_pct < 20) ? COLOR_RED : COLOR_GREEN;
  lcd_draw_string(10, 115, bat_str, bat_color, COLOR_DARK_BG, 1);

  /* Voltage */
  char mv_str[16];
  snprintf(mv_str, sizeof(mv_str), "%u.%02uV", battery_mv / 1000,
           (battery_mv % 1000) / 10);
  lcd_draw_string(85, 115, mv_str, COLOR_WHITE, COLOR_DARK_BG, 1);

  /* Event count */
  char evt_str[20];
  snprintf(evt_str, sizeof(evt_str), "EVENTS: %lu", (unsigned long)event_count);
  lcd_draw_string(10, 135, evt_str, COLOR_CYAN, COLOR_DARK_BG, 1);

  /* Bottom accent */
  lcd_fill_rect(10, 220, LCD_WIDTH - 20, 2, COLOR_ACCENT);
}

void ui_show_calibration(const char *step) {
  s_last_activity_us = esp_timer_get_time();

  lcd_fill_screen(COLOR_DARK_BG);

  /* Header */
  lcd_fill_rect(0, 0, LCD_WIDTH, 20, COLOR_ORANGE);
  const char *hdr = "CALIBRATE";
  lcd_draw_string(center_x(hdr, 1), 6, hdr, COLOR_BLACK, COLOR_ORANGE, 1);

  /* Step instruction */
  lcd_draw_string(center_x(step, 2), 80, step, COLOR_WHITE, COLOR_DARK_BG, 2);

  /* Hint */
  const char *hint = "HOLD POSITION";
  lcd_draw_string(center_x(hint, 1), 130, hint, COLOR_CYAN, COLOR_DARK_BG, 1);

  /* Progress dots */
  const char *dots = "...";
  lcd_draw_string(center_x(dots, 2), 160, dots, COLOR_ACCENT, COLOR_DARK_BG, 2);
}

void ui_show_message(const char *msg, uint16_t color) {
  s_last_activity_us = esp_timer_get_time();

  lcd_fill_screen(COLOR_DARK_BG);
  lcd_draw_string(center_x(msg, 2), 100, msg, color, COLOR_DARK_BG, 2);
}

void ui_set_display(bool on) {
  if (on && !s_display_on) {
    power_mgr_set_lcd_power(true);
    lcd_cmd(ST7789_DISPON);
    s_display_on = true;
    s_last_activity_us = esp_timer_get_time();
  } else if (!on && s_display_on) {
    lcd_cmd(0x28); /* DISPOFF */
    power_mgr_set_lcd_power(false);
    s_display_on = false;
  }
}

bool ui_is_display_on(void) {
  /* Auto-off after timeout */
  if (s_display_on) {
    int64_t elapsed = esp_timer_get_time() - s_last_activity_us;
    if (elapsed > (int64_t)DISPLAY_TIMEOUT_MS * 1000LL) {
      ui_set_display(false);
    }
  }
  return s_display_on;
}
